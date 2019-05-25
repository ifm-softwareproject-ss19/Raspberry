#include "../include/uss_servo.h"

bool initialized = false;

bool isInitialized(void) {
	return initialized;
}

void init_uss_servo(void) {
	int wps = wiringPiSetupPhys();
#ifdef DEBUG
	std::cout << "Setup Rueckgabe: " << wps << std::endl;
#endif
	pinMode(USS_GPIO_ECHO, INPUT);
	pinMode(USS_GPIO_TRIGGER, OUTPUT);
	pinMode(SERVO_GPIO, PWM_OUTPUT);
	pwmSetClock(20); //Muss evtl. auf 20 und 200 abgeändert werden (ms statt Hz)
	pwmSetRange(200);
	digitalWrite(USS_GPIO_TRIGGER, LOW);
	pullUpDnControl(USS_GPIO_ECHO, PUD_DOWN);
	//pwmWrite(SERVO_GPIO, 15);
	initialized = true;
}

void turnSensor(us_angle angle = 90) {
	if (angle > 180) return;
	else {
		int pulse = (int)round((((float)angle / 180.0f) + 1.0f) * 10.0f);
		pwmWrite(SERVO_GPIO, pulse);
	}
}

double getDistance(int tries) {
	long values = 0;
	struct timeval time;
	long start = 0;
	long stop = 0;

	if (tries < 1) tries = 1;
	if (tries > MAXTRIES) tries = MAXTRIES;

	for (int i = 0; i < tries; i++) {
		std::cout << "Starte Messung: " << micros() << std::endl;
		digitalWrite(USS_GPIO_TRIGGER, HIGH);
		//usleep(10);
		delayMicroseconds(20);
		digitalWrite(USS_GPIO_TRIGGER, LOW);

	#ifdef DEBUG
		long debug = micros();
		std::cout << "Signal: Low" << std::endl;
	#endif
		while (digitalRead(USS_GPIO_ECHO) == LOW) {
	#ifdef DEBUG
			if (micros() - debug > 5000000) {
				std::cout << "Kein Ende des LOW-Signals" << std::endl;
				break;
			}
	#endif
			gettimeofday(&time, NULL);
			start = time.tv_usec;
			//start = micros();
		}
	#ifdef DEBUG
		debug = micros();
		std::cout << "Signal: High" << std::endl;
	#endif
		while (digitalRead(USS_GPIO_ECHO) == HIGH) {
	#ifdef DEBUG
			if (micros() - debug > 5000000) {
				std::cout << "Kein Ende des HIGH-Signals" << std::endl;
				break;
			}
	#endif
			gettimeofday(&time, NULL);
			stop = time.tv_usec;
			//stop = micros();
		}

	#ifdef DEBUG
		std::cout << "Zwischenergebnis " << i+1 << ": " << stop-start << std::endl;
	#endif
		values += (stop - start);
	}

	#ifdef DEBUG
	std::cout << "Values: " << values << " -> Double: " << (double)values << std::endl;
	#endif

	return (double)values / (double)tries;

}

float runtimeToDistance(long mys) {
	float runtime = float((double)mys / 1000000.0);
	return ((runtime / 2.0f) / 1000.0) * 34.35f;
}
