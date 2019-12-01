import cv2
import config
import utils
import time
import numpy as np
from newmovement import Mainboard
import realsenseloader
from math import sqrt
from simple_pid import PID

movement = Mainboard()
realsenseloader.load_realsense()
#movement.thrower(100)
#time.sleep(1.5)
value = 150

try:
    basket_color = config.get_color_range("pink")
except KeyError:
    exit("Blue color has not been thresholded, run threshold.py")
cap = cv2.VideoCapture(2)
cv2.namedWindow("frame")
def none(x):
    pass
#speeds = cv2.createTrackbar("speed", "frame", 150, 250, none)

while cap.isOpened():
    #print("siin")
    #throwerspeed = cv2.getTrackbarPos(speeds, "frame")
    movement.thrower(value)
    print("value", value)
    #time.sleep()
    _, frame = cap.read()
    basket_mask = utils.apply_color_mask(frame, basket_color)
    find_basket = utils.find_basket(basket_mask)

    if find_basket is not None:
        (x1, y1), (w, h), (cX, cY) = find_basket

        cv2.rectangle(frame, (x1, y1), (x1 + w, y1 + h), (0, 255, 0), 2)
        print("Distance", movement.calc_distance(w))
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
    if cv2.waitKey(1) & 0xFF == ord("j"):
        value+=1
    if cv2.waitKey(1) & 0xFF == ord("h"):
        value-=1

    cv2.imshow("frame", frame)

cap.release()