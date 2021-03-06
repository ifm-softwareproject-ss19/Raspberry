from bluetooth import *
from time import time
import threading
import subprocess
from PiCar import *

import Main

# Bluetooth Server starten und in einer
# Dauerschleife auf Input warten
def runServer(car):
    while True:
        subprocess.call(["/home/pi/Desktop/bluetoothsettings.sh"])

        server_sock = BluetoothSocket( RFCOMM )
        server_sock.bind(("",PORT_ANY))
        server_sock.listen(1)

        port = server_sock.getsockname()[1]

        uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"
        
        advertise_service( server_sock, "BluetoothServer",
                           service_id = uuid,
                           service_classes = [ uuid, SERIAL_PORT_CLASS ],
                           profiles = [ SERIAL_PORT_PROFILE ],
                         )

        print("Warte auf eine Verbindung an RFCOMM Port %d" % port)
        client_sock, client_addr = server_sock.accept()
        print("Verbunden mit: ", client_addr)

        subprocess.call(["/home/pi/Desktop/bluetoothconnected.sh"])
        car.btSocket = client_sock

        while True:
            try:
                data = client_sock.recv(256).decode('utf-8')
            except IOError:
                break
            except:
                break
            if (len(data) == 0):
                break
            print(str(time()) + ": Empfangen: " + data)
            Main.btInput(data, car)
        
        car.state = State.IDLE
        car.stop()
        
        print("Verbindung wurde getrennt")
        car.btSocket = None
        client_sock.close()
        server_sock.close()
        print("Sockets wurden geschlossen")    
