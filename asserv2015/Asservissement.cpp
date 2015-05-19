#include "Asservissement.h"

#define ABS(x) 		((x) < 0 ? - (x) : (x))
#define MAX(a,b) 	((a) > (b) ? (a) : (b))
#define MIN(a,b) 	((a) < (b) ? (a) : (b))

// Constructors ////////////////////////////////////////////////////////////////

Asservissement::Asservissement()
{
	reset();
}

// Public Methods //////////////////////////////////////////////////////////////

void Asservissement::reset()
{
	// Constante de l'asservissement et du mouvement
	maxPWM = 	0;
	Kp = 		0;
	Kd = 		0;
	Vmax = 		0;
	Acc = 		0;
	trapeze =	false;

	// Consigne par défaut et position du robot à l'initialisation
	positionIntermediaire = 0;
	consigne = 0;

	positionIntermediaireZoom = 0;
	consigneZoom = 0;

	// Palier de vitesse
	n = 0;

	// Aucun blocage à l'initialisation
	blocageDetecte = 0;
	blocageTemp = 0;
	deltaPrecedent = 0;

	erreur =  0;

	// Calcul de l'erreur maximum afin de détecter les blocages
	calculeErreurMax();
}

/*
 * Calcule la puissance moteur à fournir pour atteindre la nouvelle position théorique
 */

int Asservissement::calculePwm(long int positionReelle)
{
	long int delta = (positionIntermediaire - positionReelle); // ticks

	int pwm = (Kp * delta + Kd * (delta - deltaPrecedent)) * 0.01;

	if (pwm > maxPWM) {
		pwm = maxPWM;
	} else if (pwm < -maxPWM ) {
		pwm = -maxPWM;
	}

	deltaPrecedent = delta;

	return pwm;
}

/*
 * Calcule la nouvelle position à atteindre
 */

void Asservissement::calculePositionIntermediaire(long int positionReelle)
{
	long int deltaZoom = consigneZoom - positionIntermediaireZoom; // ticksPRESCALER

	if (trapeze) {
		dFreinage = (Acc * (ABS(n) + 1) * (ABS(n) + 2)) / 2;
		if (ABS(deltaZoom) >=  dFreinage) {
			if (deltaZoom >= 0) {
				n++;
				if (Acc * n >  Vmax) {
					n--;
					if (Acc * n > Vmax)
						n--;
				}
			}
			else {
				n--;
				if (Acc * n < -  Vmax) {
					n++;
					if (Acc * n < -Vmax)
						n++;
				}
			}
		} else {
			if (n > 0) {
				n--;
			} else if (n < 0) {
				n++;
			}
		}

		erreur = positionIntermediaireZoom - positionReelle * PRESCALER;
		//positionIntermediaireZoom += n * Acc;
		// if (ABS(erreur) < erreurMax) {
			positionIntermediaireZoom += n * Acc;
			if (blocageTemp > 0) {
				blocageTemp--;
			}
			if (blocageTemp < 0) {
				blocageTemp++;
			}
		// } else if (erreur >= 0) {
		// 	positionIntermediaireZoom = positionReelle * PRESCALER + erreurMax;
		// 	if (blocageTemp < TRIGGER_BLOCAGE) {
		// 		blocageTemp++;
		// 	}
		// } else {
		// 	positionIntermediaireZoom = positionReelle * PRESCALER - erreurMax;
		// 	if (blocageTemp > -TRIGGER_BLOCAGE) {
		// 		blocageTemp--;
		// 	}
		// }
	} else {
		if (deltaZoom > Vmax) {
			positionIntermediaireZoom += Vmax;
		} else if (deltaZoom < -Vmax) {
			positionIntermediaireZoom -= Vmax;
		}
	}

	positionIntermediaire = positionIntermediaireZoom / PRESCALER;
}

/*
 * Calcule l'erreur maximum (positionReelle et positionIntermediaire) afin de déterminer le blocage
 */

void Asservissement::calculeErreurMax()
{
	erreurMax = (PRESCALER * 2 * maxPWM) / Kp;
}

/*
 * Définit la nouvelle consigne
 */

void Asservissement::changeConsigne(long int consigneDonnee)
{
	consigne = consigneDonnee;
	consigneZoom = consigneDonnee * PRESCALER;
}

/*
 * Définition dynamique des paramètres
 */

void Asservissement::changeKp(int KpDonne)
{
	Kp = KpDonne;
	calculeErreurMax();
}

void Asservissement::changeKd(int KdDonne)
{
	Kd = KdDonne;
}

void Asservissement::changePWM(unsigned char maxPwmDonne)
{
	maxPWM = maxPwmDonne;
	calculeErreurMax();
}

void Asservissement::changeAcc(long int AccDonne)
{
	Acc = AccDonne;
}

void Asservissement::changeVmax(long int VmaxDonne)
{
	Vmax = VmaxDonne;
}

void Asservissement::changeTrapeze(unsigned char trapezeDonne)
{
	trapeze = trapezeDonne;
}