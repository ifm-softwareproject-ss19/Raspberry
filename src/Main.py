from threading import Thread

import BluetoothServer
from PiCar import *
from BTInputParser import *

def main():
    car = PiCar()
    car.start()
    btServer = Thread(target = BluetoothServer.runServer, args = (car,))
    btServer.start()
    
    #btServer.join()
    #car.join()

if __name__ == "__main__":
    main()