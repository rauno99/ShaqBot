import math, serial, threading, time

class Mainboard:


    def __init__(self):
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
            if len(message) > 3:
                print(message)
            if len(message) > 17:
                message = str(message[5:17])
                fieldID = str(message[3])
                selfID = str(message[4])
                print(message)
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
        #print("forward")
        return

    def moveBack(self):
        self.ser.write(str.encode("sd:0:-10:10 \r \n"))
        #print("back")
        return

    def rotateLeftAndRight(self, x, x1):

        if x1 is not None:
            if x1 > 300:
                self.wheel1 = 30
            elif x1 < 310:
                self.wheel1 = -30
        else:
            self.wheel1 = -30

        if x > 330:
            self.wheel2 = (x-320)/2
        elif x < 310:
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

    def makeThrow(self, speed):
        Mainboard.moveForward(self)
        Mainboard.thrower(self, speed)

    def thrower(self, speed):
        self.ser.write(str.encode("d:"+str(speed) + " \r \n"))
        #print("thrower")
        return

    def throwerStop(self):
        self.ser.write(str.encode("d:"+ str(140) + " \r \n"))
        #print("thrower stop")
        return