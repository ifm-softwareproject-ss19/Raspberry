#ifndef USS_SERVO_H
#define USS_SERVO_H

#define DEBUG

#include <wiringPi.h>
#ifdef DEBUG
#include <iostream>
#endif
#include <sys/time.h>
#include <math.h>
#include <unistd.h>

//Defines fuer die benutzten GPIO Pins
#define USS_GPIO_TRIGGER 16
#define USS_GPIO_ECHO 40
#define SERVO_GPIO 12

//define for the allowed times of measurements
#define MAXTRIES 100

//Typedef fuer die Uebergabe der Winkel zum Drehen des Motors
typedef unsigned char us_angle;

//Gibt zurueck ob die Initialisierung bereits durchgelaufen ist
bool isInitialized(void);

//Funktion zum Initialisieren des Ultraschallsensors und des Servos - muss einmal ausgefuehrt werden
void init_uss_servo(void);

//Funktion zum Drehen des Servos zu einem bestimmten Winkel zwischen 0 und 180 Grad - kein Argument = Drehen zum Mittelpunkt
void turnSensor(us_angle angle);

//Funktion zum Messen der Entfernung vor dem Ultraschallsensor mit Rueckgabe in Mikrosekunden - die Messung wird tries Mal ausgefuehrt und der Durchschintt gebildet
double getDistance(int tries);

//Umrechnungsfunktion zum Umrechnen der Signallaufzeit in Mikrosekunden zur Distanz in Centimeter
float runtimeToDistance(long ms);

#endif