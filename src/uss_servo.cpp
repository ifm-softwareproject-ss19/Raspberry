#include "../include/uss_servo.h"

bool initialized = false;

void init(void) {
	wiringPiSetup();
	pinMode(USS_GPIO_ECHO, INPUT);
	pinMode(USS_GPIO_TRIGGER, OUTPUT);
	pinMode(SERVO_GPIO, PWM_OUTPUT);
	pwmSetClock(50); //Muss evtl. auf 20 und 200 abgeändert werden (ms statt Hz)
	pwmSetRange(500);
	initialized = true;
}

void turnSensor(us_angle angle = 90) {
	if (angle > 180) return -1;
	else {
		int pulse = (int)round((((float)angle / 180.0f) + 1.0f) * 10.0f);
		pwmWrite(SERVO_GPIO, pulse);
		return 0;
	}
}

long getDistance(void) {
	struct timeval time;
	long start = 0;
	long end = 0;

	digitalWrite(USS_GPIO_TRIGGER, 1);
	usleep(10);
	digitalWrite(USS_GPIO_TRIGGER, 0);
	
	while (!digitalRead(USS_GPIO_ECHO)) {
		gettimeofday(&time, NULL);
		start = time.tv_usec;
	}
	while (digitalRead(USS_GPIO_ECHO)) {
		gettimeofday(&time, NULL);
		stop = time.tv_usec;
	}

	return stop - start;
}