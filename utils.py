import cv2
import numpy as np

def readThrowerFile(failname):
    fail = open(failname)

    andmed = []
    for rida in fail:
        jupp = rida.strip().split(",")
        distance = jupp[0]
        speed = jupp[1]
        andmed.append((distance, speed))
    fail.close()
    return andmed

def get_color_range_mean(color_range):
    return tuple([
        (color_range[0][i] + color_range[1][i]) // 2
        for i in range(2)
    ])


def apply_color_mask(src, color_range):
    # Convert to HSV and apply color mask
    hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, tuple(color_range[0]), tuple(color_range[1]))

    # Remove noise (Google opencv morphological transformations)
    kernel = np.ones((3, 3), np.uint8)
    less_noise = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    return less_noise


def find_biggest_circle(src):
    contours, _hierarchy = cv2.findContours(src, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    circles = map(cv2.minEnclosingCircle, contours)
    circles = sorted(circles, key = lambda circle: circle[1])

    if len(circles):
        (x, y), radius = circles[-1]
        return (int(x), int(y)), int(radius)

def find_basket(src):
    contours_basket, _ = cv2.findContours(src, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    for contour in contours_basket:
        M = cv2.moments(contour)

        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        (x, y, w, h) = cv2.boundingRect(contour)
        if w >= 15 and h >= 15:
            return (int(x), int(y)), (int(w), int(h)), (int(cX), int(cY))
