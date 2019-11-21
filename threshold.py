import cv2
from functools import partial
import realsenseloader
import config
import utils
import numpy as np

realsenseloader.load_realsense()
#kernel = np.ones((5,5), np.uint8)

color_name = input("Color name: ")
print("Select color range and press s to save, q to quit")

try:
    color_range = config.get_color_range(color_name)
except KeyError:
    color_range = [
        [0, 0, 0],
        [179, 255, 255]
    ]


def update_range(i, j, value):
    color_range[i][j] = value

cv2.namedWindow("frame")
cv2.createTrackbar("h_min", "frame", color_range[0][0], 179, partial(update_range, 0, 0))
cv2.createTrackbar("s_min", "frame", color_range[0][1], 255, partial(update_range, 0, 1))
cv2.createTrackbar("v_min", "frame", color_range[0][2], 255, partial(update_range, 0, 2))
cv2.createTrackbar("h_max", "frame", color_range[1][0], 179, partial(update_range, 1, 0))
cv2.createTrackbar("s_max", "frame", color_range[1][1], 255, partial(update_range, 1, 1))
cv2.createTrackbar("v_max", "frame", color_range[1][2], 255, partial(update_range, 1, 2))

cap = cv2.VideoCapture(2)

while cap.isOpened():
    _, frame = cap.read()
    cv2.imshow("frame", frame)

    mask = utils.apply_color_mask(frame, color_range)
   # mask = utils.apply_black_mask(frame, color_range)
    #opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    #dilation = cv2.dilate(mask, kernel, iterations=1)

    cv2.imshow("mask", mask)

    # Keyboard input
    key = cv2.waitKey(10)

    if key & 0xFF == ord("s"):
        config.set_color_range(color_name, color_range[0], color_range[1])

    if key & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
