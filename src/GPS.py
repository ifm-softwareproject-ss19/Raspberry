import serial
import string
import pynmea2
import math
from geographiclib.geodesic import Geodesic

port = "/dev/ttyS0"
baudrate = 9600
timeout = 0.5

tolerance = 0.00003

def readGPS(car):
    ser = serial.Serial(port = port, baudrate = baudrate, timeout = timeout)
    data = ser.readline().decode('utf-8')
    while True:
        try:
            data = ser.readline().decode('utf-8')
        except serial.serialutil.SerialException:
            ser = serial.Serial(port = port, baudrate = baudrate, timeout = timeout)
        except:
            pass
        if(data[0:6] == '$GPGGA' or data[0:6] == '$GPRMC'):
            try:
                gpsData = pynmea2.parse(data)
                if(abs(car.latitude - gpsData.latitude) > tolerance or abs(car.longitude - gpsData.longitude) > tolerance):
                    car.direction = calcDirection((car.latitude, car.longitude), (gpsData.latitude, gpsData.longitude))
                    car.latitude = gpsData.latitude
                    car.longitude = gpsData.longitude
                    car.destDirection = calcDirection((car.latitude, car.longitude), (car.destLatitude, car.destLongitude))
                    print(car.latitude, car.longitude, car.direction, car.destDirection)
                    #car.btSocket.send("Auto: " + str(car.direction) + "\n")
                    #car.btSocket.send("Ziel: " + str(car.destDirection) + "\n")
                    #car.btSocket.send(str(car.direction - car.destDirection) + "\n")
                    #car.btSocket.send(str(car.direction - car.destDirection - 360) + "\n")
                    #car.btSocket.send(str(car.destDirection - car.direction) + "\n")


            except pynmea2.nmea.ChecksumError:
                pass
            except:
                pass
            
def calcDirection(pointA, pointB):
    if (type(pointA) != tuple) or (type(pointB) != tuple):
        raise TypeError("Only tuples are supported as arguments")

    return Geodesic.WGS84.Inverse(pointA[0], pointA[1], pointB[0], pointB[1])['azi1']
