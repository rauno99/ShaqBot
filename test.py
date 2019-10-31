import cv2
import config
import utils
import time
import numpy as np
from newmovement import Mainboard

#NB realsesnse is loaded with "realsense-viewer" in terminal

firstMillis = int(round(time.time()*1000))
newMillis = 0
throwerStatus = False
throwerSpeeds = utils.readThrowerFile("throwerFile.csv")
throwerSpeeds = sorted(throwerSpeeds)
count = 0
kernel = np.ones((5,5), np.uint8)
#print(throwerSpeeds)

try:
    ball_color = config.get_color_range("ball")
except KeyError:
    exit("Ball color has not been thresholded, run threshold.py")

try:
    basket_color = config.get_color_range("pink")
except KeyError:
    exit("Blue color has not been thresholded, run threshold.py")

def capWhileMoving(frame):
    while True:
        _, frame = cap.read()


cap = cv2.VideoCapture(2)

movement = Mainboard()

while cap.isOpened():

    movement.currentlyMove = True

    # Check for messages from xBee
    if movement.currentlyMove:

        width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)  # float

        millis = int(round(time.time()*1000))
        #if millis -firstMillis < 5000:
        #    ser.write(str.encode("sd:0:0:0:0"))
        #    ser.write(str.encode("\r \n"))

        _, frame = cap.read()

        ball_mask = utils.apply_color_mask(frame, ball_color)
        basket_mask = utils.apply_color_mask(frame, basket_color)

        biggest_ball = utils.find_biggest_circle(ball_mask)
        find_basket = utils.find_basket(basket_mask)

        lineThickness = 2
        #cv2.line(frame, (335, 0), (335, 480), (0, 255, 0), lineThickness)
        cv2.line(frame, (327, 0), (327, 480), (0, 255, 0), lineThickness)


        if biggest_ball is not None:
            (x, y), radius = biggest_ball
            cv2.circle(frame, (x, y), radius, utils.get_color_range_mean(ball_color), 5)
            #print(x, y)

            if y < 390:
                movement.omniDirectional(x, y)
            elif y > 410:
                movement.moveBack()
            else:
                if find_basket is not None:
                    (x1, y1), (w, h), (cX, cY) = find_basket
                    cv2.rectangle(frame, (x1, y1), (x1 + w, y1 + h), (0, 255, 0), 2)

                    frame = cv2.line(frame, (cX, 0), (cX, 700), (255, 0, 0), 5)
                    #print("width: " + str(w))
                    #print("Distance " + str(movement.distance(w)))
                    #korviga ühele joonele
                    if cX < 331 and cX > 325: #331 325
                        #print("Vahemik oige")
                        #movement.stop()
                        if not throwerStatus:
                            throwerStatus = True
                            millis = int(round(time.time() * 1000))
                            newMillis = int(round(time.time() * 1000))
                            print("ok")
                            movement.stop()
                            movement.mapping(20, 50, 1100, 1300, w)
                            #time.sleep(0.8)
                            movement.moveForward()
                            speedMin, speedMax, distanceMin, distanceMax, distance = (movement.getSpeedsFromList(throwerSpeeds, movement.distance(w)))
                            print("Distance " + str(distance))

                            throwerSpeed = movement.throwerSpeeds(speedMin, speedMax, distanceMin, distanceMax, distance)
                            #print("throwerspeed " + str(throwerSpeed))
                            if distance < 60:
                                movement.servoMaxAngle()
                            movement.thrower(throwerSpeed)
                            #movement.thrower(180)

                            #time.sleep(0.7)
                            count += 1
                            print("Throw number: " + str(count))
                            movement.servoDown()

                    else:
                        #print(x1)
                        if throwerStatus:
                            throwerStatus = False
                            movement.throwerStop()
                            movement.servoDown()
                        movement.rotateLeftAndRight(x, cX)
                        #print("vahemik vale")

                else:
                    #print("no basket")
                    movement.rotateLeftAndRight(x, x1=None)
        else:
            #print("no ball")
            movement.moveLeft()
            if throwerStatus:
                movement.throwerStop()
                movement.servoStop()
                throwerStatus = False
                #print(throwerStatus)

        #opening = cv2.morphologyEx(frame, cv2.MORPH_OPEN, kernel)
        cv2.imshow("frame", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        if cv2.waitKey(1) & 0xFF == ord("p"):
            print("Manual override")
            center = 327
            while True:

                _, frame = cap.read()
                cv2.line(frame, (center, 0), (center, 480), (0, 255, 0), lineThickness)
                cv2.imshow("frame", frame)

                if cv2.waitKey(1) & 0xFF == ord("w"):
                    movement.moveForward()
                if cv2.waitKey(1) & 0xFF == ord("s"):
                    movement.moveBack()
                if cv2.waitKey(1) & 0xFF == ord("a"):
                    movement.moveLeft()
                if cv2.waitKey(1) & 0xFF == ord("d"):
                    movement.moveRight()
                if cv2.waitKey(1) & 0xFF == ord("v"):
                    movement.directLeftRight(90)
                if cv2.waitKey(1) & 0xFF == ord("b"):
                    movement.directLeftRight(-90)
                if cv2.waitKey(1) & 0xFF == ord("f"):
                    movement.thrower(200)
                if cv2.waitKey(1) & 0xFF == ord("r"):
                    movement.throwerStop()
                if cv2.waitKey(1) & 0xFF == ord("x"):
                    movement.boost()
                if cv2.waitKey(1) & 0xFF == ord("o"):
                    center += 1
                    print(center)
                if cv2.waitKey(1) & 0xFF == ord("p"):
                    center -= 1
                    print(center)
                if cv2.waitKey(1) & 0xFF == ord("g"):
                    print("Here I go")
                    break

movement.servoStop()
cap.release()
cv2.destroyAllWindows()
