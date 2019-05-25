#include "../include/uss_servo.h"
#include <iostream>

#define TIMES 5
#define TRIES 1

using namespace std;

int main() {
	//Turntest
	cout << "Initialisiere" << endl;
	init_uss_servo();
	cout << "Teste Drehen des Sensors" << endl;
	turnSensor(10);
	delay(1);
	turnSensor(90);
	delay(1);
	turnSensor(170);
	delay(1);
	turnSensor(90);

	//Distanztest
	cout << "Messe Distanz" << endl;
	for(int i = 0; i < TIMES; i++) {
		double rt = getDistance(TRIES);
		cout << "Messung: " << endl << "Signallaufzeit: " << rt << "mys" << endl << "Distanz: " << runtimeToDistance(rt) << "cm" << endl;
		sleep(1);
	}
}
