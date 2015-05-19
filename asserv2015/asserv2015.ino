// ####################################
// Assert macro
// ####################################

#define assert(val, msg) char msg[val ? 0 : -1];

// ####################################
// Libs
// ####################################

#include <inttypes.h>
#include <FlexiTimer2.h>
#include <Metro.h>
#include "LS7366R.h"
#include "Asservissement.h"

// ####################################
// Data
// ####################################

typedef struct {
  int32_t rotationKp;
  int32_t rotationKd;
  int32_t rotationVmax;
  int8_t rotationPWM;
  int32_t rotationConsigne;
  uint8_t rotationTrapeze;
  int32_t rotationAcc;

  int32_t distanceKp;
  int32_t distanceKd;
  int32_t distanceVmax;
  int8_t distancePWM;
  int32_t distanceConsigne;
  uint8_t distanceTrapeze;
  int32_t distanceAcc;

  int16_t moteurGauche;
  int16_t moteurDroite;
  uint8_t reboot;
} Consignes;

typedef struct {
  uint8_t header1;
  uint8_t header2;
  uint8_t header3;
  int32_t codeurGauche;
  int32_t codeurDroite;
  int16_t moteurGauche;
  int16_t moteurDroite;
  uint8_t tramesIn;
  uint8_t tramesOut;
} Output;

assert(sizeof(Consignes) == 49, taille_consignes);
assert(sizeof(Output) == 17, taille_output);

// ####################################
// PINS
// ####################################

#define PIN_DOUT_LED_INTERN 6
#define PIN_DOUT_LED_EXTERN 0

#define PIN_DOUT_COMPTEUR_G 10
#define PIN_DOUT_COMPTEUR_D 9

#define PIN_DOUT_MOTEUR_PWM_G 14
#define PIN_DOUT_MOTEUR_PWM_D 15
#define PIN_DOUT_MOTEUR_DIR_G 16
#define PIN_DOUT_MOTEUR_DIR_D 19
#define PIN_DOUT_MOTEUR_ENABLE_G 17
#define PIN_DOUT_MOTEUR_ENABLE_D 20

// ####################################
// Moteurs
// ####################################

inline void moteurs_init_enable()
{
  pinMode(PIN_DOUT_MOTEUR_PWM_G, OUTPUT);
  pinMode(PIN_DOUT_MOTEUR_DIR_G, OUTPUT);
  pinMode(PIN_DOUT_MOTEUR_ENABLE_G, OUTPUT);

  pinMode(PIN_DOUT_MOTEUR_PWM_D, OUTPUT);
  pinMode(PIN_DOUT_MOTEUR_DIR_D, OUTPUT);
  pinMode(PIN_DOUT_MOTEUR_ENABLE_D, OUTPUT);

  digitalWrite(PIN_DOUT_MOTEUR_ENABLE_G, HIGH);
  digitalWrite(PIN_DOUT_MOTEUR_ENABLE_D, HIGH);
}

inline void moteur_gauche(const int16_t pwm)
{
  if (pwm >= 0) {
    analogWrite(PIN_DOUT_MOTEUR_PWM_G, pwm);
    digitalWrite(PIN_DOUT_MOTEUR_DIR_G, LOW);
  } else {
    analogWrite(PIN_DOUT_MOTEUR_PWM_G, -pwm);
    digitalWrite(PIN_DOUT_MOTEUR_DIR_G, HIGH);
  }
}

inline void moteur_droite(const int16_t pwm)
{
  if (pwm >= 0) {
    analogWrite(PIN_DOUT_MOTEUR_PWM_D, pwm);
    digitalWrite(PIN_DOUT_MOTEUR_DIR_D, HIGH);
  } else {
    analogWrite(PIN_DOUT_MOTEUR_PWM_D, -pwm);
    digitalWrite(PIN_DOUT_MOTEUR_DIR_D, LOW);
  }
}

// ####################################
// Globals
// ####################################

const int CLOCK_RATE = 5;
const int COMMUNICATION_CLOCK_OVERFLOW = 2; // 2=10ms; 10=50ms
const int LED_CLOCK_OVERFLOW = 20; // 2=10ms; 10=50ms; 20=100ms

uint8_t clock = 0;
uint8_t communication_clock = 0;
uint8_t led_clock = 0;
uint8_t led_state = 1;

int8_t bufferPosition;
Consignes consignes;
Output output;

Asservissement asservRotation;
Asservissement asservDistance;

LS7366R codeurs(PIN_DOUT_COMPTEUR_G, PIN_DOUT_COMPTEUR_D, MDR0_CONF, MDR1_CONF);

// ####################################
// Setup
// ####################################

void setup() {
  //delay(200); // attendre une tension stable
  SPI.begin();
  codeurs.config();
  codeurs.reset();
  Serial.begin(115200);

  pinMode(PIN_DOUT_LED_EXTERN, OUTPUT);
  digitalWrite(PIN_DOUT_LED_EXTERN, LOW);

  bufferPosition = -2;

  output.header1 = 170;
  output.header2 = 170;
  output.header3 = 171;

  codeurs.reset();
  moteurs_init_enable();
  FlexiTimer2::set(CLOCK_RATE, interruption_sample);
  FlexiTimer2::start();
}

void loop() {
  if (clock) {
    clock = 0;
    communication_clock++;
    led_clock++;

    asservissement();
    readSerial();

    if (communication_clock == COMMUNICATION_CLOCK_OVERFLOW) {
      communication_clock = 0;
      writeSerial();
    }

    if (led_clock == LED_CLOCK_OVERFLOW) {
      led_clock = 0;
      showLEDActivity();
    }
  }
}

void interruption_sample() {
  codeurs.sync();
  output.codeurGauche = -codeurs.left();
  output.codeurDroite = codeurs.right();

  clock++;
}

inline void showLEDActivity() {
  if (Serial.dtr()) {
    led_state = 1 - led_state;
  } else {
    led_state = 1;
  }
  digitalWrite(PIN_DOUT_LED_EXTERN, led_state);
}

// ####################################
// Communication
// ####################################

inline void readSerial(void) {
  while (Serial.available()) {
    uint8_t byte = Serial.read();

    if (bufferPosition == -2) {
      if (byte == '@') {
        bufferPosition = -1;
      } else {
        bufferPosition = -2;
      }
    } else if (bufferPosition == -1) {
      if (byte == '@') {
        bufferPosition = 0;
      } else {
        bufferPosition = -2;
      }
    } else if (bufferPosition >= 0) {
      *((uint8_t *) &consignes + bufferPosition) = byte;
      bufferPosition++;

      if (bufferPosition == sizeof(Consignes)) {
        output.tramesIn++;
        applyConsignes();
        bufferPosition = -2;
      }
    }
  }
}

inline void writeSerial() {
  output.tramesOut++;

  if (Serial.dtr()) { // attend d'être branché
    Serial.write((const uint8_t*) &output, sizeof(Output));
    Serial.send_now();
  }
}

// ####################################
// Asserv
// ####################################

inline void applyConsignes() {
  asservRotation.changeKp(consignes.rotationKp);
  asservRotation.changeKd(consignes.rotationKd);
  asservRotation.changeVmax(consignes.rotationVmax);
  asservRotation.changePWM(consignes.rotationPWM);
  asservRotation.changeConsigne(consignes.rotationConsigne);
  asservRotation.changeTrapeze(consignes.rotationTrapeze);
  asservRotation.changeAcc(consignes.rotationAcc);

  asservDistance.changeKp(consignes.distanceKp);
  asservDistance.changeKd(consignes.distanceKd);
  asservDistance.changeVmax(consignes.distanceVmax);
  asservDistance.changePWM(consignes.distancePWM);
  asservDistance.changeConsigne(consignes.distanceConsigne);
  asservDistance.changeTrapeze(consignes.distanceTrapeze);
  asservDistance.changeAcc(consignes.distanceAcc);

  if (consignes.reboot) {
    _restart_Teensyduino_();
    delay(20);
  }
}

inline void asservissement() {
  if (consignes.moteurGauche != 0 || consignes.moteurDroite != 0) {
    output.moteurGauche = consignes.moteurGauche;
    output.moteurDroite = consignes.moteurDroite;
  } else {
    long int orientation = output.codeurGauche - output.codeurDroite;
    long int distance = output.codeurGauche + output.codeurDroite;

    asservRotation.calculePositionIntermediaire(orientation);
    asservDistance.calculePositionIntermediaire(distance);

    int pwmRotation = asservRotation.calculePwm(orientation);
    int pwmDistance = asservDistance.calculePwm(distance);

    output.moteurGauche = pwmDistance + pwmRotation;
    output.moteurDroite = pwmDistance - pwmRotation;
  }

  moteur_gauche(output.moteurGauche);
  moteur_droite(output.moteurDroite);
}
