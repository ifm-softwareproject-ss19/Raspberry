#ifndef USS_SERVO_H
#define USS_SERVO_H

#include <wiringPi.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>
#include <unistd.h>

//Defines fuer die benutzten GPIO Pins
#define USS_GPIO_TRIGGER = 16
#define USS_GPIO_ECHO = 18
#define SERVO_GPIO = 12

//Typedef fuer die Uebergabe der Winkel zum Drehen des Motors
typedef unsigned char us_angle;

bool initialized;

//Funktion zum Initialisieren des Ultraschallsensors und des Servos - muss einmal ausgefuehrt werden
void init_uss_servo(void);

//Funktion zum Drehen des Servos zu einem bestimmten Winkel zwischen 0 und 180 Grad - kein Argument = Drehen zum Mittelpunkt
void turnSensor(us_angle angle);

//Funktion zum Messen der Entfernung vor dem Ultraschallsensor - Rueckgabe in Mikro
long getDistance(void);

#endif