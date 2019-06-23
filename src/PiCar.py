from threading import Thread
import RPi.GPIO as GPIO
from time import time
from enum import Enum
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
        self.__maxInputDelay = 0.5
        self.__lastManualInput = 0
        
        self.__longitude = 0.0
        self.__latitude = 0.0
        
        self.destLongitude = 0.0
        self.destLatitude = 0.0
    
        
        self.drive = Drive.STOP
        self.steering = Steering.FORWARD
        self.state = State.IDLE

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
            
            elif(self.state == State.AUTOMATIC):
                print("Fahre nach %f, %f" % (self.destLongitude, self.destLatitude))
            
            elif(self.state == State.MANUAL):
                if(time() - self.__lastManualInput < self.__maxInputDelay):
                    print(time(), self.__lastManualInput, self.__maxInputDelay)
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
                    print("ManualDrive Timeout")
                    self.__stopDrive()
                    self.__stopSteer()
                
            elif(self.state == State.IDLE):
                pass
            
            elif(self.state == State.ERROR):
                pass
            
            
    # Vorwärts fahren
    def __forward(self):
        GPIO.output(self.__motorDriveForwardPin, GPIO.HIGH)
        GPIO.output(self.__motorDriveBackwardPin, GPIO.LOW)
        print("Vorwärts fahren")

    # Rückwärts fahren
    def __backward(self):
        GPIO.output(self.__motorDriveForwardPin, GPIO.LOW)
        GPIO.output(self.__motorDriveBackwardPin, GPIO.HIGH)
        print("Rückwärts fahren")

    # Nach links lenken
    def __left(self):
        GPIO.output(self.__motorSteerLeftPin, GPIO.HIGH)
        GPIO.output(self.__motorSteerRightPin, GPIO.LOW)
        print("Links fahren")

    # Nach rechts lenken
    def __right(self):
        GPIO.output(self.__motorSteerLeftPin, GPIO.LOW)
        GPIO.output(self.__motorSteerRightPin, GPIO.HIGH)
        print("Rechts fahren")

    # Fahren stoppen
    def __stopDrive(self):
        GPIO.output(self.__motorDriveForwardPin, GPIO.LOW)
        GPIO.output(self.__motorDriveBackwardPin, GPIO.LOW)
        print("Fahren stoppen")

    # Lenken stoppen
    def __stopSteer(self):
        GPIO.output(self.__motorSteerLeftPin, GPIO.LOW)
        GPIO.output(self.__motorSteerRightPin, GPIO.LOW)
        print("Lenken stoppen")

    # Alles stoppen
    def stop(self):
        self.state = State.IDLE
        GPIO.output(self.__motorDriveForwardPin, GPIO.LOW)
        GPIO.output(self.__motorDriveBackwardPin, GPIO.LOW)
        GPIO.output(self.__motorSteerLeftPin, GPIO.LOW)
        GPIO.output(self.__motorSteerRightPin, GPIO.LOW)

    def startManualDrive(self):
        print(self.state)
        self.state = State.MANUAL
        print(self.state)

    def stopManualDrive(self):
        self.state = State.IDLE

    def manualInput(self):
        self.__lastManualInput = time()

    def setDestGPS(self, longitude, latitude):
        self.destLongitude = longitude
        self.destLatitude = latitude

    # Konstanten für automatisches Ausweichen
    tries = 3
    minSpace =
    tryAngles = [90, 45, 75, 115, 135]

    # Messvorgang: Distanz vor Wagen
    def measure():
        space = 0
        if tries < 1: tries = 1
        for i in range(tries):
            spaces += Uss.getDistance()
        space /= tries
        return space

    # Distanz vor Wagen messen und bei Bedarf Objekten ausweichen
    def objectAvoiding(cont):
        if cont == False:
            space = measure()
            if space < minSpace: cont = True
        
        if cont == True:
            self.stop()
            i = 0
            measurements = []
            for angle in tryAngles:
                measurements[i] = measure()
                i++

            bestMatch = [-1, 100000]
            for i in range(len(tryAngles)):
                Servo.turnServo(tryAngles[i])
                time.sleep(1)
                if measurements[i] < bestMatch[1]:
                    bestMatch[0] = i
                    bestMatch[1] = measurements[i]
            Servo.turnServo(90)
            time.sleep(1)
                    
            if bestMatch[0] == -1: return False
            elif bestMatch[1] < minSpace:
                self.__backward()
                sleep(0.7)
                self.stop()
                objectAvoiding(True)
            else:
                #turn car to tryAngle[bestMatch[0]]
                self.__forward()
                if objectAvoiding(False) == False: return False
                else:
                    #turn car back
                    if measure() > minSpace:
                        self.__forward()
                        return objectAvoiding(False)

        else: return True
