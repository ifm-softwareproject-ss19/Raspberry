import serial
import string
import pynmea2
import math

port = "/dev/ttyS0"
baudrate = 9600
timeout = 0.5

tolerance = 0.0005

def readGPS(car):
    ser = serial.Serial(port = port, baudrate = baudrate, timeout = timeout)
    data = ser.readline().decode('utf-8')
    while True:
        try:
            data = ser.readline().decode('utf-8')
            #print(data)
        except serial.serialutil.SerialException:
            ser = serial.Serial(port = port, baudrate = baudrate, timeout = timeout)
        except:
            pass
        if(data[0:6] == '$GPGGA' or data[0:6] == '$GPRMC'):
            try:
                gpsData = pynmea2.parse(data)
                if(abs(car.latitude - gpsData.latitude) > tolerance or abs(car.longitude - gpsData.longitude) > tolerance):
                    car.direction = calcDirection((gpsData.latitude, gpsData.longitude), (car.latitude, car.longitude))
                    car.latitude = gpsData.latitude
                    car.longitude = gpsData.longitude
                    car.destDirection = calcDirection((car.latitude, car.longitude), (car.destLatitude, car.destLongitude))
                    print(car.latitude, car.longitude, car.direction, car.destDirection)
            except pynmea2.nmea.ChecksumError:
                pass
            except:
                pass
            
def calcDirection(pointA, pointB):
    if (type(pointA) != tuple) or (type(pointB) != tuple):
        raise TypeError("Only tuples are supported as arguments")

    lat1 = math.radians(pointA[0])
    lat2 = math.radians(pointB[0])

    diffLong = math.radians(pointB[1] - pointA[1])

    x = math.sin(diffLong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
            * math.cos(lat2) * math.cos(diffLong))

    initial_bearing = math.atan2(x, y)

    # Now we have the initial bearing but math.atan2 return values
    # from -180° to + 180° which is not what we want for a compass bearing
    # The solution is to normalize the initial bearing as shown below
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360

    return compass_bearing