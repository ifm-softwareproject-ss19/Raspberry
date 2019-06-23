import Uss
import Servo

tries = 3
minSpace =
tryAngles = [90, 45, 75, 115, 135]

def measure():
    space = 0
    if tries < 1: tries = 1
    for i in range(tries):
        spaces += Uss.getDistance()
    space /= tries
    return space

def objectAvoiding(cont):
    if cont == False:
        space = measure()
        if space < minSpace: cont = True
    
    if cont == True:
        #brake
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
            #drive backwards
            objectAvoiding(True)
        else:
            #turn car to tryAngle[bestMatch[0]]
            #drive forward
            if objectAvoiding(False) == False: return False
            else:
                #turn car back
                if measure() > minSpace:
                    #drive forward
                    if objectAvoiding(False): return False
                    else: return True

    else: return True
