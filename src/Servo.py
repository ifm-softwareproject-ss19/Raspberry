import RPi.GPIO as GPIO
import time

servo = 10
sleeptime = 0.7

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo,GPIO.OUT)
# in servo motor,
# 1ms pulse for 0 degree (LEFT)
# 1.5ms pulse for 90 degree (MIDDLE)
# 2ms pulse for 180 degree (RIGHT)

# so for 50hz, one frequency is 20ms
# duty cycle for 0 degree = (1/20)*100 = 5%
# duty cycle for 90 degree = (1.5/20)*100 = 7.5%
# duty cycle for 180 degree = (2/20)*100 = 10%

p = GPIO.PWM(servo,50)# 50hz frequency
p.start(6.6)# starting duty cycle ( it set the servo to 0 degree )
time.sleep(sleeptime)
p.ChangeDutyCycle(0.0)

def turnServo(angle):
    if angle > -1 and angle < 181:
        turnVal = 2.2 + (8.8 * (angle / 180))
        print("Servo drehen: Grad: ", angle, ", turnVal: ", turnVal)
        try:
            #p.start(turnVal)
            p.ChangeDutyCycle(turnVal)
            time.sleep(sleeptime)
            p.ChangeDutyCycle(0.0)
        except KeyboardInterrupt:
            GPIO.cleanup()