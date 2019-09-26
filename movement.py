import math

def moveLeft(ser):
    ser.write(str.encode("sd:-10:-10:-10 \r \n"))
    print("left")
    return

def moveRight(ser):
    ser.write(str.encode("sd:10:10:10 \r \n"))
    print("right")
    return

def moveForward(ser):
    ser.write(str.encode("sd:0:10:-10 \r \n"))
    print("right")
    return

def stop(ser):
    ser.write(str.encode("sd:0:0:0 \r \n"))
    print("stop")
    return

def omniDirectional(ser, x, y):
    robotSpeed = 20
    wheelAngle1 = 0
    wheelAngle2 = 240
    wheelAngle3 = 120

    yLine = 480 - y
    xLine = 320 - x


    #robotDirectionAngle calcualted from x and y coords of ball
    robotDirectionAngle = int(math.degrees(math.atan((320 - x)/y)) + 90)
    print(robotDirectionAngle)

    wheelLinearVelocity1 = int(robotSpeed * math.cos(math.radians(robotDirectionAngle - wheelAngle1)))
    wheelLinearVelocity2 = int(robotSpeed * math.cos(math.radians(robotDirectionAngle - wheelAngle2)))
    wheelLinearVelocity3 = int(robotSpeed * math.cos(math.radians(robotDirectionAngle - wheelAngle3)))

    print(wheelLinearVelocity1, wheelLinearVelocity2, wheelLinearVelocity3)

    ser.write(str.encode("sd:" + str(wheelLinearVelocity1) + ":" + str(wheelLinearVelocity3) + ":" + str(wheelLinearVelocity2) + " \r \n"))
    return
