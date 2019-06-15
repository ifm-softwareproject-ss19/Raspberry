import re
from time import time

from PiCar import *

def btInput(input, car):
    if(re.search("^\s*automaticDrive\s*\(\s*\-?[0-9]+\.[0-9]+\s*,\s*\-?[0-9]+\.[0-9]+\s*\)\s*$", input)): # automaticDrive(GPS)
        gps = re.findall("\-?[0-9]+\.[0-9]+", input)
        car.setDestGPS(float(gps[0]), float(gps[1]))
        car.state = State.AUTOMATIC
        print(car.state)
        
    elif(re.search("^\s*getGpsData\s*\(\s*\)\s*$", input)): # getGpsData()
        print("Sende aktuelle GPS Daten...")
        
    elif(re.search("^\s*startGpsData\s*\(\s*[0-9]+\s*\)\s*$", input)): # startGpsData(int interval)
        interval = float(re.findall("[0-9]+", input)[0])
        print("Sende alle %d Sekunden GPS Daten" % (interval))
        
    elif(re.search("^\s*stopGpsData\s*\(\s*\)\s*$", input)): # stopGpsData()
        print("Stoppe senden von GPS Daten")
    
    elif(re.search("^\s*startManualDrive\s*\(\s*\)\s*$", input)): # startManualDrive()
        car.startManualDrive()
    
    elif(re.search("^\s*stopManualDrive\s*\(\s*\)\s*$", input)): # stopManualDrive()
        car.stopManualDrive()
        
    elif(re.search("^\s*emergencyStop\s*\(\s*\)\s*$", input)): # emergencyStop()
        car.stop()
        car.state = State.IDLE
        print("Not-Halt")
        
    elif(re.search("^\s*continueDriving\s*\(\s*\)\s*$", input)): # continueDriving()
        car.setState = State.AUTOMATIC
        print("Weiter zum Ziel fahren")
     
    elif(re.search("^\s*manualDirection\s*\(\s*stop|front|back\s*,\s*forward|left|right\s*\)\s*$", input)): # manualDirection(Drive direction1, Steering direction2)
        drive = re.findall("stop|front|back", input)[0]
        steering = re.findall("forward|left|right", input)[0]
        car.drive = Drive[drive.upper()]
        car.steering = Steering[steering.upper()]
        car.manualInput()
        print("Manuelle Richtung: %s, %s" % (car.drive, car.steering))
        
    elif(re.search("^\s*getStatus\s*\(\s*\)\s*$", input)): # getStatus()
        print("Sende Status")

    else:
        print("Ungültige Eingabe...")