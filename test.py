import cv2
import config
import utils
import serial
import time
import movement

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

try:
    ball_color = config.get_color_range("ball")
except KeyError:
    exit("Ball color has not been thresholded, run threshold.py")

try:
    basket_color = config.get_color_range("blue")
except KeyError:
    exit("Blue color has not been thresholded, run threshold.py")

cap = cv2.VideoCapture(2)

while cap.isOpened():


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
    cv2.line(frame, (335, 0), (335, 480), (0, 255, 0), lineThickness)
    cv2.line(frame, (305, 0), (305, 480), (0, 255, 0), lineThickness)

    while ser.inWaiting() > 0:
        ser.readline()
        ser.flush()

    if biggest_ball is not None:
        (x, y), radius = biggest_ball
        cv2.circle(frame, (x, y), radius, utils.get_color_range_mean(ball_color), 5)
        #print(x, y)

        movement.omniDirectional(ser, x, y)


    cv2.drawContours(frame, find_basket, -1, (255, 0, 0), 3)
    cv2.imshow("frame", frame)

    if cv2.waitKey(10) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
