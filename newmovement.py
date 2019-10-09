import math, serial, threading, time

class Mainboard:

    def __init__(self):
        self.W = 16
        self.F = (104 * 100) / self.W
        self.wheel1 = 0
        self.wheel2 = 0
        self.robotSpeed = 60
        self.wheelAngle1 = 0
        self.wheelAngle2 = 240
        self.wheelAngle3 = 120
        self.currentlyMove = False
        self.setFieldID = "A"
        self.setSelfID = "A"
        self.message = ""
        print("overwrite")
        self.xbeethread = threading.Thread(target=self.messenger, daemon=True)
        self.ser = serial.Serial("/dev/ttyACM0", timeout = 0.01, baudrate = 9600)
        self.xbeethread.start()

    def messenger(self):
        while True:
            setFieldID = self.setFieldID
            setSelfID = self.setSelfID
            message = self.ser.read(19)
            if len(message) > 17:
                message = str(message[5:17])
                fieldID = str(message[3])
                selfID = str(message[4])
                #print(message)
                self.ser.flush()
                if fieldID == setFieldID and (selfID == setSelfID or selfID == "X"):
                    if "START" in message:
                        self.ser.write(str.encode('rf:a' + fieldID + setSelfID + 'ACK----- \r \n'))
                        print("start")
                        self.currentlyMove = True
                    elif "STOP" in message:
                        self.ser.write(str.encode('rf:a' + fieldID + setSelfID + 'ACK----- \r \n'))
                        print("Stop!")
                        self.currentlyMove =  False
                    elif "PING" in message:
                        print("DOS")
                        self.ser.write(str.encode('rf:a' + fieldID + setSelfID + 'ACK----- \r \n'))
                        self.currentlyMove
                    else:
                        self.currentlyMove
                else:
                    self.currentlyMove
            else:
                self.currentlyMove

    def moveLeft(self):
        self.ser.write(str.encode("sd:-10:-10:-10 \r \n"))
        #print("left")current
        return

    def moveRight(self):
        self.ser.write(str.encode("sd:10:10:10 \r \n"))
        #print("right")
        return

    def moveForward(self):
        self.ser.write(str.encode("sd:0:30:-30 \r \n"))
        print("forward")
        return

    def boost(self):
        self.ser.write(str.encode("sd:0:100:-100 \r \n"))
        return

    def moveBack(self):
        self.ser.write(str.encode("sd:0:-10:10 \r \n"))
        #print("back")
        return
    def rotateLeftAndRight(self, x, x1):

        if x1 is not None:
            if x1 > 310:
                self.wheel1 = 15
            elif x1 < 330:
                self.wheel1 = -15
        else:
            self.wheel1 = -30

        if x > 350:
            self.wheel2 = (x-320)/2
        elif x < 290:
            self.wheel2 = (x-320)/2
        else:
            self.wheel2 = 0
        self.ser.write(str.encode("sd:" + str(self.wheel1) + ":"+ str(self.wheel2) +":"+ str(self.wheel2) +" \r \n"))
        return

    def rotateRight(self):
        self.ser.write(str.encode("sd:20:0:0 \r \n"))
        return

    def stop(self):
        self.ser.write(str.encode("sd:0:0:0 \r \n"))
        #qprint("stop")
        return

    def directLeftRight(self, angle):
        #-90 is RIGHT, 90 is LEFT

        robotDirectionAngle = int(math.degrees(math.atan((320)) + angle))

        wheelLinearVelocity1 = int(-self.robotSpeed * math.cos(math.radians(robotDirectionAngle - self.wheelAngle1)))
        wheelLinearVelocity2 = int(-self.robotSpeed * math.cos(math.radians(robotDirectionAngle - self.wheelAngle2)))
        wheelLinearVelocity3 = int(-self.robotSpeed * math.cos(math.radians(robotDirectionAngle - self.wheelAngle3)))

        # print(wheelLinearVelocity1, wheelLinearVelocity2, wheelLinearVelocity3)

        self.ser.write(str.encode("sd:" + str(wheelLinearVelocity1) + ":" + str(wheelLinearVelocity2) + ":" + str(
            wheelLinearVelocity3) + " \r \n"))


    def omniDirectional(self, x, y):

        #robotDirectionAngle calcualted from x and y coords of ball
        try:
            robotDirectionAngle = int(math.degrees(math.atan((320 - x)/y)) + 90)
        except ZeroDivisionError:
            robotDirectionAngle = 0.1

        #print(robotDirectionAngle)

        wheelLinearVelocity1 = int(-self.robotSpeed * math.cos(math.radians(robotDirectionAngle - self.wheelAngle1)))
        wheelLinearVelocity2 = int(-self.robotSpeed * math.cos(math.radians(robotDirectionAngle - self.wheelAngle2)))
        wheelLinearVelocity3 = int(-self.robotSpeed * math.cos(math.radians(robotDirectionAngle - self.wheelAngle3)))

        #print(wheelLinearVelocity1, wheelLinearVelocity2, wheelLinearVelocity3)

        self.ser.write(str.encode("sd:" + str(wheelLinearVelocity1) + ":" + str(wheelLinearVelocity2) + ":" + str(wheelLinearVelocity3) + " \r \n"))
        return

    def thrower(self, speed):
        self.ser.write(str.encode("d:"+str(speed) + " \r \n"))
        #print("thrower")
        return

    def throwerStop(self):
        self.ser.write(str.encode("d:"+ str(140) + " \r \n"))
        #print("thrower stop")
        return

    def mapping(self, widthMin, widthMax, angleMin, angleMax, basketWidth):
        # Basically arduino map function
        try:
            print("basketwidth"+str(basketWidth))
            widthSpan = widthMax - widthMin
            angleSpan = angleMax - angleMin

            valueScaled = float(basketWidth - widthMin) / float(widthSpan)
            throwerAngleCalculation = int(angleMin + (valueScaled * angleSpan))
            print("ThrowerAngle" + str(throwerAngleCalculation))
            self.ser.write(str.encode("sv:" + str(throwerAngleCalculation) + " \r \n"))
        except NameError:
            print("error")

    def getSpeedsFromList(self, listSpeeds, distance):
        for x in listSpeeds:
            xZero = float(x[0])
            xOne = float(x[1])

            if xZero < distance:
                distanceMin = xZero
                speedMin = xOne
            elif xZero > distance:
                distanceMax = xZero
                speedMax = xOne
                print("results " + str(speedMin) + " " + str(speedMax) + " " + str(distanceMin) + " " + str(distanceMax) + " " + str(distance))
                return speedMin, speedMax, distanceMin, distanceMax, distance



    def throwerSpeeds(self, distanceMin, distanceMax, speedMin, speedMax, distance):
        # Basically arduino map function
        try:
            print("distance " + str(distance))
            widthSpan = speedMax - speedMin
            angleSpan = distanceMax - distanceMin

            valueScaled = float(distance - speedMin) / float(widthSpan)
            throwerSpeed = int(distanceMin + (valueScaled * angleSpan))

            if throwerSpeed > 0:
                print("throwerSpeed2 " + str(throwerSpeed))
                return throwerSpeed
        except NameError:
            print("error")

    def distance(self, width):
        distance = round((self.W * self.F) / width ,2)
        return distance


    def servoStop(self):
        self.ser.write(str.encode("sv:" + str(600) + " \r \n"))
        return