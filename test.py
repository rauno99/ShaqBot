import cv2
import config
import utils
import serial
import time
import movement
import numpy as np
import xbeeListener
import threading, queue
#NB realsesnse is loaded with "realsense-viewer" in terminal

ser = serial.Serial("/dev/ttyACM0")
ser.baudrate = 115200
ser.stopbits = 1
ser.timeout = 0.1
ser.bytesize = 8
ser.setDTR(1)
ser.parity = "N"
ser.write(str.encode("\r \n"))
firstMillis = int(round(time.time()*1000))
newMillis = 0

global currentlyMove

throwerStatus = False
currentlyMove = False
actualMove = False
q = queue.Queue()

setSelfID = "A"
setFieldID = "A"

kernel = np.ones((5,5), np.uint8)

try:
    ball_color = config.get_color_range("ball")
except KeyError:
    exit("Ball color has not been thresholded, run threshold.py")

try:
    basket_color = config.get_color_range("blue")
except KeyError:
    exit("Blue color has not been thresholded, run threshold.py")

cap = cv2.VideoCapture(2)

def xbeeThread(ser, setFieldID, setSelfID, currentlyMove, q):
    import xbeeListener
    while True:
        currentlyMove = xbeeListener.messenger(ser, setFieldID, setSelfID, currentlyMove)
        q.put(currentlyMove)
        print(currentlyMove)

xbeeThreadStarter = threading.Thread(target=xbeeThread, args=(ser, setFieldID, setSelfID, currentlyMove, q))
xbeeThreadStarter.start()

while cap.isOpened():
    # Check for messages from xBee
    currentlyMove = q.get()
    #currentlyMove = xbeeListener.messenger(ser, setFieldID, setSelfID, currentlyMove)
    if currentlyMove == False:
        while True:
            #currentlyMove = xbeeListener.messenger(ser, setFieldID, setSelfID, currentlyMove)
            currentlyMove = q.get()
            if currentlyMove == True:
                break

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
    #cv2.line(frame, (305, 0), (305, 480), (0, 255, 0), lineThickness)

    while ser.inWaiting() > 0:
        ser.flush()

    if biggest_ball is not None:
        (x, y), radius = biggest_ball
        cv2.circle(frame, (x, y), radius, utils.get_color_range_mean(ball_color), 5)
        #print(x, y)

        if y < 350:
            movement.omniDirectional(ser, x, y)
        else:
            if find_basket is not None:
                (x1, y1), w, h = find_basket
                cv2.rectangle(frame, (x1, y1), (x1 + w, y1 + h), (0, 255, 0), 2)

                #korviga ühele joonele
                if x1 < 310 and x1 > 300:
                    print("Vahemik oige")
                    #print("stop")
                    movement.stop(ser)
                    newMillis = int(round(time.time()*1000))
                    firstMillis = int(round(time.time()*1000))
                    throwerStatus = False
                    while newMillis - firstMillis < 1000:
                        newMillis = int(round(time.time()*1000))
                        print(str(throwerStatus) + " duringMillis")
                        if throwerStatus == False:
                            movement.throwerStop(ser)
                            movement.thrower(ser, 200)
                            throwerStatus = True
                            print(throwerStatus)
                            movement.moveForward(ser)


                else:
                    print(x1)
                    movement.rotateLeftAndRight(ser, x, x1)
                    print("vahemik vale")

            else:
                print("no basket")
                movement.rotateLeftAndRight(ser, x, x1=None)
    else:
        #print("no ball")
        movement.moveLeft(ser)
        if throwerStatus == True:
            movement.throwerStop(ser)
            throwerStatus = False
            print(throwerStatus)

    #opening = cv2.morphologyEx(frame, cv2.MORPH_OPEN, kernel)
    cv2.imshow("frame", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
    if cv2.waitKey(1) & 0xFF == ord("p"):
        print("Manual override")
        while True:
            _, frame = cap.read()
            cv2.imshow("frame", frame)

            if cv2.waitKey(1) & 0xFF == ord("w"):
                movement.moveForward(ser)
            if cv2.waitKey(1) & 0xFF == ord("s"):
                movement.moveBack(ser)
            if cv2.waitKey(1) & 0xFF == ord("a"):
                movement.moveLeft(ser)
            if cv2.waitKey(1) & 0xFF == ord("d"):
                movement.moveRight(ser)
            if cv2.waitKey(1) & 0xFF == ord("v"):
                movement.directLeftRight(ser, 90)
            if cv2.waitKey(1) & 0xFF == ord("b"):
                movement.directLeftRight(ser, -90)
            if cv2.waitKey(1) & 0xFF == ord("f"):
                movement.thrower(ser, 200)
            if cv2.waitKey(1) & 0xFF == ord("r"):
                movement.throwerStop(ser)
            if cv2.waitKey(1) & 0xFF == ord("g"):
                print("Here I go")
                break


cap.release()
cv2.destroyAllWindows()
