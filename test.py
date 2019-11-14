import cv2
import config
import utils
import time
import numpy as np
from newmovement import Mainboard
import realsenseloader
from simple_pid import PID

#NB realsesnse is loaded with "realsense-viewer" in terminal

realsenseloader.load_realsense()

firstMillis = int(round(time.time()*1000))
newMillis = 0
throwerStatus = False
throwerSpeeds = utils.readThrowerFile("throwerFile.csv")
throwerSpeeds = sorted(throwerSpeeds)
count = 0
kernel = np.ones((5,5), np.uint8)
circling = True
#print(throwerSpeeds)

try:
    ball_color = config.get_color_range("ball")
except KeyError:
    exit("Ball color has not been thresholded, run threshold.py")

try:
    basket_color = config.get_color_range("pink")
except KeyError:
    exit("Blue col)or has not been thresholded, run threshold.py")

try:
    black_line_color = config.get_color_range("black")
except KeyError:
    exit("Blue col)or has not been thresholded, run threshold.py")

pid = PID(0.7, 0, 0.01, setpoint=302)


cap = cv2.VideoCapture(2)

movement = Mainboard()

while cap.isOpened():

    #movement.currentlyMove = True

    # Check for messages from xBee
    if movement.currentlyMove:

        #width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)  # float

        _, frame = cap.read()

        ball_mask = utils.apply_color_mask(frame, ball_color)
        basket_mask = utils.apply_color_mask(frame, basket_color)
        #black_line_mask = utils.apply_color_mask(frame, black_line_color)

        biggest_ball = utils.find_biggest_circle(ball_mask)
        find_basket = utils.find_basket(basket_mask)
        #find_black_line = utils.find_black_line(black_line_mask)

        lineThickness = 2
        #cv2.line(frame, (335, 0), (335, 480), (0, 255, 0), lineThickness)
        cv2.line(frame, (327, 0), (327, 480), (0, 255, 0), lineThickness)


        if biggest_ball is not None:
            (x, y), radius = biggest_ball
            cv2.circle(frame, (x, y), radius, utils.get_color_range_mean(ball_color), 5)

            #print(x, y)

            if y < 350 and circling is True:
                movement.omniDirectional(x, y)
            elif y < 390 and y > 350 and circling is True:
                movement.slowOmniDirectional(x, y)
            elif y > 410 and circling is True:
                movement.moveBack()
            else:
                if find_basket is not None:
                    (x1, y1), (w, h), (cX, cY) = find_basket
                    cv2.rectangle(frame, (x1, y1), (x1 + w, y1 + h), (0, 255, 0), 2)
                    if movement.calc_distance(w) < 50:
                        movement.moveLeft()
                        time.sleep(0.5)

                    frame = cv2.line(frame, (cX, 0), (cX, 700), (255, 0, 0), 5)
                    #print("width: " + str(w))
                    #print("Distance " + str(movement.distance(w)))
                    #korviga ühele joonele
                    firstMillis = int(round(time.time() * 1000))
                    #print("firstmillis")
                    #if cX < 331 and cX > 325 and circling is True: #331 325
                    if cX > 300 and cX < 304 and circling is True:

                        #print("Vahemik oige")
                        #movement.stop()
                        #print("thing")
                        #print("ok")
                        movement.stop()
                        movement.mapping(350, 450, 1950, 2100, movement.calc_distance(w))
                        movement.getThrowerSpeed(w)
                        #movement.thrower(190)
                        print("Distance", movement.calc_distance(w))
                        print(movement.throwerspeed)
                        circling = False

                        #movement.getThrowerSpeed(w)

                        #print("Distance " + str(movement.calc_distance(w)))
                        #print("throwerspeed " + str(throwerSpeed))

                        #if distance < 60:
                        #    movemenqt.servoMaxAngle()

                        count += 1
                        #print("Throw number: " + str(count))
                        #movement.servoDown()


                    elif circling == False:
                        while True:
                            movement.moveForward()
                        #v = int(pid(cX))
                        #print("V " + str(v))
                            #millis = int(round(time.time() * 1000))
                        #if cX < 345 and cX > 335:
                            #movement.rotateLeftAndRight(x, cX)
                            #millis2 = int(round(time.time() * 1000))
                        #else:
                        #if millis - firstMillis > 100:
                        #movement.moveSlowForward()
                        #    movement.moveForwardPID(-v)

                        #else:
                        #    movement.stop()
                            time.sleep(1)
                            #if millis - firstMillis > 2000:
                            #print("here?")
                            circling = True
                            movement.throwerStop()
                            movement.servoDown()
                            movement.stop()
                            _, frame = cap.read()
                            break

                    else:
                        #print(x1)
                        wheel1Speed = int(pid(cX))
                        if wheel1Speed > 20:
                            wheel1Speed = 20
                        movement.rotateLeftAndRight(x, cX, -wheel1Speed)
                        #print("vahemik vale")

                else:
                    #print("no basket")
                    movement.rotateLeftAndRight(x, x1=None, wheel1Speed=None)
                    circling = True
                    movement.throwerStop()
                    movement.servoDown()
        else:
            #print("no ball")
            movement.moveLeft()

        #opening = cv2.morphologyEx(frame, cv2.MORPH_OPEN, kernel)
        cv2.imshow("frame", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        if cv2.waitKey(1) & 0xFF == ord("p"):
            movement.stop()
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
