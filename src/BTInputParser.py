import re
from time import sleep
import threading
from threading import Thread
from GPS import calcDirection

from PiCar import *

intervalSender = None
condition = threading.Condition()
stop_thread = False

def getStopThread():
    global stop_thread
    return stop_thread

def intervalSend(interval, car):
    getGpsData(car)
    global stop_thread
    stop_thread = False
    while not stop_thread:
        with condition:
            if(not condition.wait_for(getStopThread, interval)):
                getGpsData(car)

def btInput(input, car):
    if(re.search("^\s*automaticDrive\s*\(\s*\-?[0-9]+\.[0-9]+\s*,\s*\-?[0-9]+\.[0-9]+\s*\)\s*$", input)): # automaticDrive(GPS)
        gps = re.findall("\-?[0-9]+\.[0-9]+", input)
        latitude = float(gps[0])
        longitude = float(gps[1])
        automaticDrive(car, latitude, longitude)
        
    elif(re.search("^\s*getGpsData\s*\(\s*\)\s*$", input)): # getGpsData()
        getGpsData(car)
        
    elif(re.search("^\s*startGpsData\s*\(\s*[0-9]+\s*\)\s*$", input)): # startGpsData(int interval)
        interval = float(re.findall("[0-9]+", input)[0])
        if(interval > 0):
            startGpsData(interval, car)
        else:
            getGpsData(car)
        
    elif(re.search("^\s*stopGpsData\s*\(\s*\)\s*$", input)): # stopGpsData()
        stopGpsData()
    
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
     
    elif(re.search("^\s*manualDirection\s*\(\s*stop|front|back\s*,\s*forward|left|right\s*\)\s*$", input, flags = re.IGNORECASE)): # manualDirection(Drive direction1, Steering direction2)
        drive = re.findall("stop|front|back", input, flags = re.IGNORECASE)[0]
        steering = re.findall("forward|left|right", input, flags = re.IGNORECASE)[0]
        car.drive = Drive[drive.upper()]
        car.steering = Steering[steering.upper()]
        car.manualInput()
        print("Manuelle Richtung: %s, %s" % (car.drive, car.steering))
        
    elif(re.search("^\s*getStatus\s*\(\s*\)\s*$", input)): # getStatus()
        print("Sende Status")

    else:
        print("Ungültige Eingabe...")

def automaticDrive(car, latitude, longitude):
    car.destLatitude = latitude
    car.destLongitude = longitude
    car.destDirection = calcDirection((car.latitude, car.longitude), (car.destLatitude, car.destLongitude))
    car.state = State.AUTOMATIC
    print(car.state)

def getGpsData(car):
    try:
        car.btSocket.send("sendLocation(" + str(car.latitude) + ", " + str(car.longitude) + ")")
        print("Sende aktuelle GPS Daten...")
    except:
        stopGpsData()
    
def startGpsData(interval, car):
    global intervalSender
    while(intervalSender != None and intervalSender.is_alive()):
        stopGpsData()
    intervalSender = Thread(target = intervalSend, args = (interval, car))
    intervalSender.start()
    print("Sende alle %d Sekunden GPS Daten" % (interval))
    
def stopGpsData():
    global condition
    global stop_thread
    with condition:
        stop_thread = True
        condition.notify()
    print("Stoppe senden von GPS Daten")
