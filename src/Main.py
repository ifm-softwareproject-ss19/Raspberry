from threading import Thread

import BluetoothServer
from PiCar import *
from BTInputParser import *
import GPS

def main():
    car = PiCar()
    car.start()
    btServer = Thread(target = BluetoothServer.runServer, args = (car,))
    btServer.start()
    gpsReader = Thread(target = GPS.readGPS, args = (car,))
    gpsReader.start()
    
    #btServer.join()
    #car.join()
    #gpsReader.join()

if __name__ == "__main__":
    main()