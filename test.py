import cv2
import config
import utils
import serial
import time

ser = serial.Serial('COM7')  # open serial port
ser.baudrate = 115200
ser.stopbits = 1
ser.bytesize = 8
ser.setDTR(1)
ser.parity = 'N'
ser.write(str.encode('\r \n'))

firstMillis = int(round(time.time() * 1000))

try:
    ball_color = config.get_color_range("ball")
except KeyError:
    exit("Ball color has not been thresholded, run threshold.py")

try:
    basket_color = config.get_color_range("blue")
except KeyError:
    exit("Blue color has not been thresholded, run threshold.py")

cap = cv2.VideoCapture(1)

while cap.isOpened():
    _, frame = cap.read()

    millis = int(round(time.time() * 1000))
    if millis - firstMillis < 10000:
        ser.write(str.encode('sd:0:5:-5:0'))  # check which port was really used
        ser.write(str.encode('\r \n'))

    ball_mask = utils.apply_color_mask(frame, ball_color)
    basket_mask = utils.apply_color_mask(frame, basket_color)

    biggest_ball = utils.find_biggest_circle(ball_mask)
    find_basket = utils.find_basket(basket_mask)

    if biggest_ball is not None:
        (x, y), radius = biggest_ball
        cv2.circle(frame, (x, y), radius, utils.get_color_range_mean(ball_color), 5)

    cv2.drawContours(frame, find_basket, -1, (255, 0, 0), 3)
    cv2.imshow("frame", frame)

    if cv2.waitKey(10) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()