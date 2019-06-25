from threading import Thread
import RPi.GPIO as GPIO
from time import time, sleep
from enum import Enum
from GPS import calcDirection
import Uss
import Servo

class Drive(Enum):
    STOP = 1
    FRONT = 2
    BACK = 3

class Steering(Enum):
    FORWARD = 1
    LEFT = 2
    RIGHT = 3

class State(Enum):
    DESTINATION = 1
    AUTOMATIC = 2
    MANUAL = 3
    IDLE = 4
    ERROR = 5

class PiCar(Thread):
    
    def __init__(self):
        super(PiCar, self).__init__()
        
        self.latitude = 0.0
        self.longitude = 0.0
        self.direction = 0.0
        
        self.destLatitude = 0.0
        self.destLongitude = 0.0
        self.destDirection = 0.0
        
        self.drive = Drive.STOP
        self.steering = Steering.FORWARD
        self.state = State.IDLE
        
        self.__gpsTolerance = 0.00005
        self.__directionTolerance = 1
        
        self.__maxInputDelay = 0.5
        self.__lastManualInput = 0
        self.__turnTime = 2
        self.__turnCooldown = 2
        self.__turnTimer = 0
        self.__turnCooldownTimer = 0
        
        # Konstanten für automatisches Ausweichen
        self.__tries = 3 # pro try ~0,02 sek benötigt
        self.__minSpace = 2 # ms x 34,5cm/ms
        self.__tryAngles = [90, 45, 75, 115, 135]

        # Pins für den Motor
        self.__motorDriveForwardPin = 4 #7
        self.__motorDriveBackwardPin = 17 #11
        self.__motorSteerLeftPin = 27 #13
        self.__motorSteerRightPin = 22 #15
        
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.__motorDriveForwardPin, GPIO.OUT)
        GPIO.setup(self.__motorDriveBackwardPin, GPIO.OUT)
        GPIO.setup(self.__motorSteerLeftPin, GPIO.OUT)
        GPIO.setup(self.__motorSteerRightPin, GPIO.OUT)
        self.stop()

    def run(self):
        while True:
            if(self.state == State.DESTINATION):
                self.stop()
                print("sende Ziel erreicht")
                self.state = State.IDLE
            
            elif(self.state == State.AUTOMATIC):
                #print("Fahre nach %f, %f" % (self.destLongitude, self.destLatitude))
                self.__automaticDrive()
            
            elif(self.state == State.MANUAL):
                self.__manualDrive()
                
            elif(self.state == State.IDLE):
                pass
            
            elif(self.state == State.ERROR):
                pass
            
            
    # Vorwärts fahren
    def __forward(self):
        GPIO.output(self.__motorDriveForwardPin, GPIO.HIGH)
        GPIO.output(self.__motorDriveBackwardPin, GPIO.LOW)
        #print("Vorwärts fahren")

    # Rückwärts fahren
    def __backward(self):
        GPIO.output(self.__motorDriveForwardPin, GPIO.LOW)
        GPIO.output(self.__motorDriveBackwardPin, GPIO.HIGH)
        #print("Rückwärts fahren")

    # Nach links lenken
    def __left(self):
        #if(GPIO.input(self.__motorSteerRightPin) == GPIO.LOW and GPIO.input(self.__motorSteerLeftPin) == GPIO.LOW):
            #GPIO.output(self.__motorSteerLeftPin, GPIO.LOW)
            #GPIO.output(self.__motorSteerRightPin, GPIO.HIGH)
        GPIO.output(self.__motorSteerLeftPin, GPIO.HIGH)
        GPIO.output(self.__motorSteerRightPin, GPIO.LOW)
        #print("Links fahren")

    # Nach rechts lenken
    def __right(self):
        #if(GPIO.input(self.__motorSteerRightPin) == GPIO.LOW and GPIO.input(self.__motorSteerLeftPin) == GPIO.LOW):
            #GPIO.output(self.__motorSteerLeftPin, GPIO.HIGH)
            #GPIO.output(self.__motorSteerRightPin, GPIO.LOW)
        GPIO.output(self.__motorSteerLeftPin, GPIO.LOW)
        GPIO.output(self.__motorSteerRightPin, GPIO.HIGH)
        #print("Rechts fahren")

    # Fahren stoppen
    def __stopDrive(self):
        GPIO.output(self.__motorDriveForwardPin, GPIO.LOW)
        GPIO.output(self.__motorDriveBackwardPin, GPIO.LOW)
        #print("Fahren stoppen")

    # Lenken stoppen
    def __stopSteer(self):
        GPIO.output(self.__motorSteerLeftPin, GPIO.LOW)
        GPIO.output(self.__motorSteerRightPin, GPIO.LOW)
        #print("Lenken stoppen")

    # Alles stoppen
    def stop(self):
        self.state = State.IDLE
        GPIO.output(self.__motorDriveForwardPin, GPIO.LOW)
        GPIO.output(self.__motorDriveBackwardPin, GPIO.LOW)
        GPIO.output(self.__motorSteerLeftPin, GPIO.LOW)
        GPIO.output(self.__motorSteerRightPin, GPIO.LOW)

    def startManualDrive(self):
        self.state = State.MANUAL
        print(self.state)

    def stopManualDrive(self):
        self.state = State.IDLE

    def manualInput(self):
        self.__lastManualInput = time()
    
    def __manualDrive(self):
        if(time() - self.__lastManualInput < self.__maxInputDelay):
            #print(time(), self.__lastManualInput, self.__maxInputDelay)
            if(self.drive == Drive.STOP):
                self.__stopDrive()
            elif(self.drive == Drive.FRONT):
                self.__forward()
            elif(self.drive == Drive.BACK):
                self.__backward()
                
            if(self.steering == Steering.FORWARD):
                self.__stopSteer()
            elif(self.steering == Steering.LEFT):
                self.__left()
            elif(self.steering == Steering.RIGHT):
                self.__right()
        else:
            #print("ManualDrive Timeout")
            self.__stopDrive()
            self.__stopSteer()
    
    def __automaticDrive(self):
        if(self.__objectAvoiding(False)):
            if(abs(self.latitude - self.destLatitude) < self.__gpsTolerance and abs(self.longitude - self.destLongitude) < self.__gpsTolerance):
                self.stop()
                self.state = State.DESTINATION
                
            elif(abs(self.direction - self.destDirection) > self.__directionTolerance and self.__turnTimer < time() and self.__turnCooldownTimer > time()):
                if((self.direction - self.destDirection + 360) % 360 > 180):
                    self.__forward()
                    self.__left()
                else:
                    self.__forward()
                    self.__right()
                self.__turnTimer = time() + ((self.direction - self.destDirection + 360) % 180) / 180 * self.__turnTime
                self.__turnCooldownTimer = self.__turnTimer + self.__turnCooldown
                
            elif((abs(self.latitude - self.destLatitude) > self.__gpsTolerance or abs(self.longitude - self.destLongitude) > self.__gpsTolerance) and self.__turnTimer < time()):
                self.__forward()
                self.__stopSteer()
        else:
            self.state = State.Error
    # Messvorgang: Distanz vor Wagen
    def __measure(self):
        space = 0
        if self.__tries < 1: tries = 1
        for i in range(self.__tries):
            space += Uss.getDistance()
            if i < self.__tries: sleep(0.02)
        space /= self.__tries
        return space * 1000 / 2
    
     # Distanz vor Wagen messen und bei Bedarf Objekten ausweichen
    def __objectAvoiding(self, cont):
        if cont == False:
            space = self.__measure()
            if space < self.__minSpace: cont = True
        
        if cont == True:
            self.stop()
            i = 0
            measurements = {}
            for angle in self.__tryAngles:
                measurements[i] = self.__measure()
                i = i + 1

            bestMatch = [-1, 100000]
            for i in range(len(self.__tryAngles)):
                Servo.turnServo(self.__tryAngles[i])
                sleep(1)
                if measurements[i] < bestMatch[1]:
                    bestMatch[0] = i
                    bestMatch[1] = measurements[i]
            Servo.turnServo(90)
            sleep(1)
                    
            if bestMatch[0] == -1: return False
            elif bestMatch[1] < self.__minSpace:
                self.__backward()
                sleep(0.7)
                self.stop()
                self.__objectAvoiding(True)
            else:
                #turn car to tryAngle[bestMatch[0]]
                drivingtime = 0
                if self.__tryAngles[bestMatch[0]] == 90: drivingtime = 2
                elif self.__tryAngles[bestMatch[0]] < 90:
                    self.__left()
                    drivingtime = (self.__turnTime / 2) * ((self.__tryAngles[bestMatch[0]] - 5) / 90)
                else:
                    self.__right()
                    drivingtime = (self.__turnTime / 2) * ((self.__tryAngles[bestMatch[0]]  - 85) / 90)
                        
                self.__forward()
                sleep(drivingtime)
                self.__stopSteer()
                if self.__objectAvoiding(False) == False: return False
                else:
                    # avoided Object
                    if self.__measure() > self.__minSpace:
                        self.__forward()
                        return self.__objectAvoiding(False)

        else: return True
