import cv2
import config
import utils
import time
import numpy as np
from newmovement import Mainboard
import realsenseloader
from math import sqrt
from simple_pid import PID

#NB realsesnse is loaded with "realsense-viewer" in terminal

#TODO
attackBasket = "pink"

realsenseloader.load_realsense()

firstMillis = int(round(time.time()*1000))
newMillis = 0
throwerStatus = False
throwerSpeeds = utils.readThrowerFile("throwerFile.csv")
throwerSpeeds = sorted(throwerSpeeds)
count = 0
kernel = np.ones((5,5), np.uint8)
circling = True
amIOutside = False
#print(throwerSpeeds)

try:
    ball_color = config.get_color_range("ball")
except KeyError:
    exit("Ball color has not been thresholded, run threshold.py")

try:
    basket_color = config.get_color_range(attackBasket)
except KeyError:
    exit("Blue color has not been thresholded, run threshold.py")

try:
    black_line_color = config.get_color_range("black")
    white_line_color = config.get_color_range("white")
except KeyError:
    exit("Blue col)or has not been thresholded, run threshold.py")

pid = PID(0.8, 0, 0.00001, setpoint=330)
pid.output_limits = (-30, 30)
toBallSpeed = PID(0.3, 0.00001, 0, setpoint=420)
rotateForBasketSpeed = PID(0.3, 0, 0, setpoint=320)
rotateForBallDuringOmni = PID(0.35, 0, 0, setpoint=320)

lastStepTimer = int(round(time.time() * 1000))
lastBallFoundTime = int(round(time.time() * 1000))

cap = cv2.VideoCapture(2)

movement = Mainboard()

frame_times = []
start_t = time.time()

while cap.isOpened():

    end_t = time.time()
    time_taken = end_t - start_t
    start_t = end_t
    frame_times.append(time_taken)
    frame_times = frame_times[-20:]
    fps = len(frame_times) / sum(frame_times)
    #print(fps)

    movement.currentlyMove = True

    # Check for messages from xBee
    if movement.currentlyMove:

        #width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)  # float
        _, frame = cap.read()

        ball_mask = utils.apply_color_mask(frame, ball_color)
        basket_mask = utils.apply_color_mask(frame, basket_color)
        black_line_mask = utils.apply_black_or_white_mask(frame, black_line_color)
        white_line_mask = utils.apply_black_or_white_mask(frame, white_line_color)

        biggest_ball = utils.find_biggest_circle(ball_mask)
        find_basket = utils.find_basket(basket_mask)
        find_black_line = utils.find_black_line(black_line_mask)
        find_white_line = utils.find_white_line(white_line_mask)

        lineThickness = 2
        #cv2.line(frame, (335, 0), (335, 480), (0, 255, 0), lineThickness)
        #cv2.line(frame, (327, 0), (327, 480), (0, 255, 0), lineThickness)

        if biggest_ball is not None:

            lastBallFoundTime = int(round(time.time() * 1000))
            try:
                (x1, y1), (w, h), (cX, cY) = find_basket
                if cX > 330:
                    movement.isBasketLeft = False
                elif cX < 310:
                    movement.isBasketLeft = True
            except:
                pass

            (x, y), radius = biggest_ball
            cv2.line(frame, (320, 480), (x, y), (0, 255, 0), lineThickness)
            if find_black_line is not None:


                #blackLine
                (lineXblack, lineYblack), (lineWblack, lineHblack), rect_angle_black, rect_black = find_black_line
                boxBlack = cv2.boxPoints(rect_black)
                boxBlack = np.int0(boxBlack)
                if lineWblack > lineHblack:
                    #cv2.line(frame, (boxBlack[0][0], boxBlack[0][1]), (boxBlack[3][0], boxBlack[3][1]), (0, 255, 255), 20)
                    cv2.line(frame, (boxBlack[0][0], boxBlack[0][1]), (boxBlack[3][0], boxBlack[3][1]), (0, 255, 255), 20)
                    squareLine = [[boxBlack[0][0], boxBlack[0][1]], [boxBlack[3][0], boxBlack[3][1]]]
                else:
                    #print("siin2")
                    cv2.line(frame, (boxBlack[2][0], boxBlack[2][1]), (boxBlack[3][0], boxBlack[3][1]), (0, 255, 255), 20)
                    squareLine = [[boxBlack[2][0], boxBlack[2][1]], [boxBlack[3][0], boxBlack[3][1]]]

                # whiteLine
                if find_white_line is not None:
                    (lineXwhite, lineYwhite), (lineWwhite, lineHwhite), rect_angle_white, rect_white = find_white_line
                    boxWhite = cv2.boxPoints(rect_white)
                    boxWhite = np.int0(boxWhite)

                    def calculateDistance(x1, y1, x2, y2):
                        dist = sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                        return dist

                    distanceBetweenBlackandWhite = calculateDistance(lineXblack, lineYblack, lineXwhite, lineYwhite)
                    distanceBetweenBlackandRobot = calculateDistance(lineXblack, lineYblack, 320, 480)
                    distanceBetweenWhiteandRobot = calculateDistance(lineXwhite, lineYwhite, 320, 480)

                    cv2.drawContours(frame, [boxWhite], 0, (255, 0, 255), 2)

                    if distanceBetweenBlackandRobot < distanceBetweenWhiteandRobot:
                        amIOutside = True
                    else:
                        amIOutside = False

                cv2.drawContours(frame, [boxBlack], 0, (0, 0, 255), 2)

                fromRobotToBallLine = [[320, 480], [x, y]]
                intersection = utils.intersectionFinder(fromRobotToBallLine, squareLine)
                #intersection = utils.line_intersection(fromRobotToBallLine, squareLine)
                #print(fromRobotToBallLine, squareLine)
                print("Am I outside? " + str(amIOutside))
                if intersection == True and amIOutside == False:
                    movement.stop()
                    movement.moveLeft()
                    time.sleep(0.15)

            cv2.circle(frame, (x, y), radius, utils.get_color_range_mean(ball_color), 5)
            #print(x, y)
            omniWheelSpeed = int(toBallSpeed(y))
            omniWheel1Speed = int(rotateForBallDuringOmni(x))
            if y < 390 and circling is True:
                movement.omniDirectional(x, y, omniWheelSpeed, omniWheel1Speed)
            #elif y < 390 and y > 330 and circling is True:
           #     movement.slowOmniDirectional(x, y)
            elif y > 410 and circling is True:
                movement.moveBack()
            else:
                wheel2Speed = int(-rotateForBasketSpeed(x))
                #print("rotateforBasketSpeed" + str(x))
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
                    #print("firstmillis")
                    #if cX < 331 and cX > 325 and circling is True: #331 325

                    if cX >= 326 and cX <= 334 and circling is True:

                        millis = int(round(time.time() * 1000))
                        #print("millisThing " + str(millis - firstMillis))
                        #print("Vahemik oige")
                        #movement.stop()
                        #print("thing")
                        #print("ok")
                        movement.stop()
                        if millis - firstMillis > 200:
                            #movement.mapping(350, 450, 1950, 2100, movement.calc_distance(w))
                            movement.getThrowerSpeed(w)
                            print("Distance", movement.calc_distance(w))
                            #movement.thrower(200)
                            #print("movement.throwerspeed " + str(movement.throwerspeed))
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
                            #print("got here")
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
                        firstMillis = int(round(time.time() * 1000))
                        #print(x1)
                        wheel1Speed = int(pid(cX))
                        #print("wheelSpeed " + str(wheel1Speed))
                        if wheel1Speed > 50:
                            wheel1Speed = 50
                        movement.rotateLeftAndRight(x, cX, -wheel1Speed, wheel2Speed)
                        #print("Cx" + str(cX))
                        #print("vahemik vale")
                else:
                    #print("no basket"
                    movement.rotateLeftAndRight(x, x1=None, wheel1Speed=None, wheel2Speed=wheel2Speed)
                    circling = True
                    movement.throwerStop()
                    movement.servoDown()
        else:
            #print("no ball")
            stepTimer = int(round(time.time() * 1000))
            timeNow = int(round(time.time() * 1000))
            #print("TIMER " + str(stepTimer - lastStepTimer))
            if stepTimer - lastStepTimer > 500 and stepTimer - lastStepTimer < 800:
                #print("STOP")
                movement.stop()
            elif stepTimer - lastStepTimer > 800:
                lastStepTimer = int(round(time.time() * 1000))
                #print("TIMER RESET")
            else:
                if timeNow - lastBallFoundTime > 300:
                    #print("TURN LEFT")
                    movement.moveLeft()
                else:
                    movement.stop()
            #movement.moveLeft()

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
