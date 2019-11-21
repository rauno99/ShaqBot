import math, serial, threading, time, utils, scipy.interpolate

class Mainboard:

    def communication(self):
        while True:
            time.sleep(0.03)
            vastus = self.ser.read(19)
            #print("vastus: " + str(vastus))
            #print(len(vastus))
            if len(vastus) > 17:
                #print("vastus: " + str(vastus))
                self.messenger(vastus)
            else:
                self.ser.flush()
            #print("wheel1", self.wheel1, "wheel2", self.wheel2, "wheel3", self.wheel3)
            text = ("sd:" + str(self.wheel1) + ":" + str(self.wheel2) + ":" + str(self.wheel3) + "\r \n")
            #print("siin")
            self.ser.write(text.encode('utf-8'))
            time.sleep(0.02)
            #print("throwerSpeed" + str(self.throwerspeed))

            if self.throwerspeed != self.lastThrowerSpeed:
                #print("throwTime")
                self.ser.write(str.encode('d:' + str(self.throwerspeed) + '\r \n'))
            self.lastThrowerSpeed = self.throwerspeed
            time.sleep(0.02)
            self.ser.write(("sv:" + str(self.throwerangle) + "\r \n").encode("utf-8"))
            #print("ThrowerAngle" + str(self.throwerangle))
            if self.currentlyMove == False:
                self.wheel1 = 0
                self.wheel2 = 0
                self.wheel3 = 0
                self.throwerspeed = 0


    def __init__(self):
        self.isBasketLeft = True
        self.throwerSpeedsList = sorted(utils.readThrowerFile("throwerFile.csv"))
        self.W = 16
        self.F = (108 * 100) / self.W
        self.wheel1 = 0
        self.wheel2 = 0
        self.wheel3 = 0
        self.throwerspeed = 0
        self.lastThrowerSpeed = 0
        self.throwerangle = 0
        self.robotSpeed = 80
        self.slowRobotSpeed = 20
        self.wheelAngle1 = 0
        self.wheelAngle2 = 240
        self.wheelAngle3 = 120
        self.distance = 0
        self.currentlyMove = False
        self.setFieldID = "B"
        self.setSelfID = "B"
        self.message = ""
        print("overwrite")
        self.communicationThread = threading.Thread(target=self.communication, daemon=True)
        self.ser = serial.Serial("/dev/ttyACM0", timeout=0.01, baudrate = 9600)
        self.communicationThread.start()

    def messenger(self, message_in):
        setFieldID = self.setFieldID
        setSelfID = self.setSelfID
        #print("messengeris", message_in)
        if len(message_in) > 17:
            message_in = str(message_in[5:17])
            fieldID = str(message_in[3])
            selfID = str(message_in[4])
            #print("stuff " + str(fieldID) + " " + str(selfID))
            #print("\n")
            self.ser.flush()
            #print(message)
            if fieldID == setFieldID and (selfID == setSelfID or selfID == "X"):
                if "START" in message_in:
                    self.ser.write(str.encode('rf:a' + fieldID + setSelfID + 'ACK----- \r \n'))
                    print("start")
                    self.currentlyMove = True
                elif "STOP" in message_in:
                    self.ser.write(str.encode('rf:a' + fieldID + setSelfID + 'ACK----- \r \n'))
                    print("Stop!")
                    self.currentlyMove = False
                elif "PING" in message_in:
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
        self.wheel1 = -30
        self.wheel2 = -30
        self.wheel3 = -30

    def moveRight(self):
        self.wheel1 = 10
        self.wheel2 = 10
        self.wheel3 = 10

    def moveForward(self):
        self.wheel1 = 0
        self.wheel2 = 30
        self.wheel3 = -30

    def moveSlowForward(self):
        self.wheel1 = 0
        self.wheel2 = 20
        self.wheel3 = -20

    def boost(self):
        self.wheel1 = 0
        self.wheel2 = 100
        self.wheel3 = 100

    def moveBack(self):
        self.wheel1 = 0
        self.wheel2 = -15
        self.wheel3 = 15

    def rotateLeftAndRight(self, x, x1, wheel1Speed, wheel2Speed):
        if x1 is not None:
            #if x1 > 325: #325
            self.wheel1 = wheel1Speed
            #elif x1 < 329: #329
            #self.wheel1 = wheel1Speed
        else:
            if self.isBasketLeft == True:
                self.wheel1 = -35
            else:
                self.wheel1 = 35

        self.wheel2 = wheel2Speed
        #print(wheel2Speed)
        #if x > 330: #337
        #    self.wheel2 = (x-320)/4
        #elif x < 320: #317
        #    self.wheel2 = (x-320)/4
        #else:
        #    self.wheel2 = 0

        self.wheel3 = self.wheel2

    def rotateRight(self):
        self.wheel1 = 20
        self.wheel2 = 0
        self.wheel3 = 0

    def stop(self):
        self.wheel1 = 0
        self.wheel2 = 0
        self.wheel3 = 0

    def directLeftRight(self, angle):
        #-90 is RIGHT, 90 is LEFT

        robotDirectionAngle = int(math.degrees(math.atan((327)) + angle))

        wheelLinearVelocity1 = int(-self.robotSpeed * math.cos(math.radians(robotDirectionAngle - self.wheelAngle1)))
        wheelLinearVelocity2 = int(-self.robotSpeed * math.cos(math.radians(robotDirectionAngle - self.wheelAngle2)))
        wheelLinearVelocity3 = int(-self.robotSpeed * math.cos(math.radians(robotDirectionAngle - self.wheelAngle3)))

        # print(wheelLinearVelocity1, wheelLinearVelocity2, wheelLinearVelocity3)
        self.wheel1 = wheelLinearVelocity1
        self.wheel2 = wheelLinearVelocity2
        self.wheel3 = wheelLinearVelocity3


    def omniDirectional(self, x, y, speed, omniWheel1Speed):

        #robotDirectionAngle calcualted from x and y coords of ball
        try:
            robotDirectionAngle = int(math.degrees(math.atan((327 - x)/y)) + 90)
        except ZeroDivisionError:
            robotDirectionAngle = 0.1

        #print(robotDirectionAngle)

        wheelLinearVelocity1 = int(-speed * math.cos(math.radians(robotDirectionAngle - self.wheelAngle1)))
        wheelLinearVelocity2 = int(-speed * math.cos(math.radians(robotDirectionAngle - self.wheelAngle2)))
        wheelLinearVelocity3 = int(-speed * math.cos(math.radians(robotDirectionAngle - self.wheelAngle3)))

        #print(wheelLinearVelocity1, wheelLinearVelocity2, wheelLinearVelocity3)

        self.wheel1 = wheelLinearVelocity1/2
        #self.wheel1 = omniWheel1Speed
        #self.wheel1 = 0
        self.wheel2 = wheelLinearVelocity2
        self.wheel3 = wheelLinearVelocity3

    def slowOmniDirectional(self, x, y):

        #robotDirectionAngle calcualted from x and y coords of ball
        try:
            robotDirectionAngle = int(math.degrees(math.atan((327 - x)/y)) + 90)
        except ZeroDivisionError:
            robotDirectionAngle = 0.1

        #print(robotDirectionAngle)

        wheelLinearVelocity1 = int(-self.slowRobotSpeed * math.cos(math.radians(robotDirectionAngle - self.wheelAngle1)))
        wheelLinearVelocity2 = int(-self.slowRobotSpeed * math.cos(math.radians(robotDirectionAngle - self.wheelAngle2)))
        wheelLinearVelocity3 = int(-self.slowRobotSpeed * math.cos(math.radians(robotDirectionAngle - self.wheelAngle3)))

        #print(wheelLinearVelocity1, wheelLinearVelocity2, wheelLinearVelocity3)

        self.wheel1 = wheelLinearVelocity1
        self.wheel2 = wheelLinearVelocity2
        self.wheel3 = wheelLinearVelocity3


    def thrower(self, speed):
        self.throwerspeed = speed

    def throwerStop(self):
        self.throwerspeed = 50

    def servoMaxAngle(self):
        self.throwerangle = 1000

    def mapping(self, widthMin, widthMax, angleMin, angleMax, basketWidth):
        # Basically arduino map function
        try:
            #print("basketwidth"+str(basketWidth))
            widthSpan = widthMax - widthMin
            angleSpan = angleMax - angleMin

            valueScaled = float(basketWidth - widthMin) / float(widthSpan)
            self.throwerangle = int(angleMin + (valueScaled * angleSpan))
        except NameError:
            print("error")

    def getThrowerSpeed(self, w):
        distance = self.calc_distance(w)
        for x in self.throwerSpeedsList:
            xZero = float(x[0])
            xOne = float(x[1])
            if xZero < distance:
                distanceMin = xZero
                speedMin = xOne
            elif xZero > distance:
                distanceMax = xZero
                speedMax = xOne
                print("results " + str(speedMin) + " " + str(speedMax) + " " + str(distanceMin) + " " + str(distanceMax) + " " + str(distance))
                #print("x: ", xZero, xOne)
        try:
            #print("distance " + str(distance))
            #print(speedMin,speedMax, distanceMin, distanceMax)
            scale = scipy.interpolate.interp1d([distanceMin, distanceMax], [speedMin, speedMax])
            # widthSpan = speedMax - speedMin
            # angleSpan = distanceMax - distanceMin
            #
            # valueScaled = float(distance - speedMin) / float(widthSpan)
            # print(widthSpan, angleSpan, valueScaled)
            # throwerSpeed = int(distanceMin + (valueScaled * angleSpan))
            throwerSpeed = int(scale(distance))

             #if distance < 60:
            #     self.throwerspeed = 180

            if throwerSpeed > 0:
                print("throwerSpeed2 " + str(throwerSpeed))
                self.throwerspeed = throwerSpeed
        except NameError:
            print("error")

    def calc_distance(self, width):
        self.distance = round((self.W * self.F) / width, 2)
        return self.distance

    def servoStop(self):
        self.throwerangle = 600

    def servoDown(self):
        self.throwerangle = 1300


    def moveForwardPID(self, wheel1):
        self.wheel1 = wheel1
        self.wheel2 = 20
        self.wheel3 = -20
