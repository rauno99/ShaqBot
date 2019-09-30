import math

def moveLeft(ser):
    ser.write(str.encode("sd:-10:-10:-10 \r \n"))
    #print("left")
    return

def moveRight(ser):
    ser.write(str.encode("sd:10:10:10 \r \n"))
    #print("right")
    return

def moveForward(ser):
    ser.write(str.encode("sd:0:30:-30 \r \n"))
    #print("forward")
    return

def moveBack(ser):
    ser.write(str.encode("sd:0:-10:10 \r \n"))
    #print("back")
    return

def rotateLeftAndRight(ser, x, x1):
    wheel1 = 0

    if x1 is not None:
        if x1 > 300:
            wheel1 = 40
        elif x1 < 310:
            wheel1 = -40
    else:
        wheel1 = -40

    if x > 330:
        wheel2 = (x-320)/2
    elif x < 310:
        wheel2 = (x-320)/2
    else:
        wheel2 = 0
    ser.write(str.encode("sd:" + str(wheel1) + ":"+ str(wheel2) +":"+ str(wheel2) +" \r \n"))
    return

def rotateRight(ser):
    ser.write(str.encode("sd:20:0:0 \r \n"))
    return

def stop(ser):
    ser.write(str.encode("sd:0:0:0 \r \n"))
    #qprint("stop")
    return

def directLeftRight(ser, angle):
    #-90 is RIGHT, 90 is LEFT

    robotSpeed = 20
    wheelAngle1 = 0
    wheelAngle2 = 240
    wheelAngle3 = 120

    robotDirectionAngle = int(math.degrees(math.atan((320)) + angle))

    wheelLinearVelocity1 = int(-robotSpeed * math.cos(math.radians(robotDirectionAngle - wheelAngle1)))
    wheelLinearVelocity2 = int(-robotSpeed * math.cos(math.radians(robotDirectionAngle - wheelAngle2)))
    wheelLinearVelocity3 = int(-robotSpeed * math.cos(math.radians(robotDirectionAngle - wheelAngle3)))

    # print(wheelLinearVelocity1, wheelLinearVelocity2, wheelLinearVelocity3)

    ser.write(str.encode("sd:" + str(wheelLinearVelocity1) + ":" + str(wheelLinearVelocity2) + ":" + str(
        wheelLinearVelocity3) + " \r \n"))


def omniDirectional(ser, x, y):
    robotSpeed = 50
    wheelAngle1 = 0
    wheelAngle2 = 240
    wheelAngle3 = 120

    #robotDirectionAngle calcualted from x and y coords of ball
    try:
        robotDirectionAngle = int(math.degrees(math.atan((320 - x)/y)) + 90)
    except ZeroDivisionError:
        robotDirectionAngle = 0.1

    #print(robotDirectionAngle)

    wheelLinearVelocity1 = int(-robotSpeed * math.cos(math.radians(robotDirectionAngle - wheelAngle1)))
    wheelLinearVelocity2 = int(-robotSpeed * math.cos(math.radians(robotDirectionAngle - wheelAngle2)))
    wheelLinearVelocity3 = int(-robotSpeed * math.cos(math.radians(robotDirectionAngle - wheelAngle3)))

    #print(wheelLinearVelocity1, wheelLinearVelocity2, wheelLinearVelocity3)

    ser.write(str.encode("sd:" + str(wheelLinearVelocity1) + ":" + str(wheelLinearVelocity2) + ":" + str(wheelLinearVelocity3) + " \r \n"))
    return

def thrower(ser, speed):
    ser.write(str.encode("d:"+str(speed) + " \r \n"))
    #print("thrower")
    return

def throwerStop(ser):
    ser.write(str.encode("d:"+ str(140) + " \r \n"))
    #print("thrower stop")
    return