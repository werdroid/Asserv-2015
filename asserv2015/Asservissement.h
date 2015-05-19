#ifndef Asservissement_h
#define Asservissement_h

#define PRESCALER	32

#define TRIGGER_BLOCAGE	15

class Asservissement{
	public:
		// Consigne et position du robot (point de vue Arduino)
		long int 	consigne; // ticks
		long int 	positionIntermediaire; // ticks

		// Consigne et position du robot zoomé
		long int 	consigneZoom; // ticksPRESCALER
		long int 	positionIntermediaireZoom; // ticksPRESCALER

		// Constantes de l'asservissement et du moteur
		long int 	Kp; // en pwm/tick
		long int	Kd; // en pwm/tick
		long int	deltaPrecedent; // en tick
		long int 	Acc; // tickPRESCALER/t2
		long int 	Vmax; // ticksPRESCALER/t
		unsigned char 	maxPWM;
		unsigned char	trapeze;

		// Distance de freinage
		long int 	dFreinage;

		// Palier de vitesse
		long int 	n;

		// Erreur maximum (sert à détecter les obstacles)
		long int 	erreurMax; // ticksPRESCALER
		long int	erreur; // ticksPRESCALER

		// Vaut 1 ou -1 si le moteur est bloqué
		int 	blocageDetecte;
		int		blocageTemp;

		Asservissement();
		void	reset();

		int 	calculePwm(long int);
		void 	calculePositionIntermediaire(long int);
		void 	calculeErreurMax();

		void	changeConsigne(long int);
		void 	changeKp(int);
		void	changeKd(int);
		void 	changeAcc(long int);
		void	changeVmax(long int);
		void 	changePWM(unsigned char);
		void	changeTrapeze(unsigned char);
};

#endif
