import cv2
import numpy as np
from shapely.geometry import LineString

def readThrowerFile(failname):
    fail = open(failname)

    andmed = []
    for rida in fail:
        jupp = rida.strip().split(",")
        #print(jupp)
        distance = int(jupp[0])
        speed = int(jupp[1])
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

def apply_black_or_white_mask(src, color_range):
    # Convert to HSV and apply color mask
    hsv = cv2.cvtColor(src, cv2.COLOR_BGR2RGB)
    mask = cv2.inRange(hsv, tuple(color_range[0]), tuple(color_range[1]))

    # Remove noise (Google opencv morphological transformations)
    kernel = np.ones((3, 3), np.uint8)
    less_noise = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    return less_noise

def find_white_line(src):
    contours_black_line, _ = cv2.findContours(src, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    for contour in contours_black_line:
        M = cv2.moments(contour)

        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        (x, y), (w, h), angle = cv2.minAreaRect(contour)
        rect = cv2.minAreaRect(contour)
        #print(str(w*h))
        if w*h > 10000:
            return (int(x), int(y)), (int(w), int(h)), int(angle), rect

def find_black_line(src):
    contours_black_line, _ = cv2.findContours(src, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    for contour in contours_black_line:
        M = cv2.moments(contour)

        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        (x, y), (w, h), angle = cv2.minAreaRect(contour)
        rect = cv2.minAreaRect(contour)
        #print(str(w*h))
        if w*h > 8000:
            return (int(x), int(y)), (int(w), int(h)), int(angle), rect

def find_biggest_circle(src):
    contours, _hierarchy = cv2.findContours(src, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    circles = map(cv2.minEnclosingCircle, contours)
    circles = sorted(circles, key = lambda circle: circle[1])

    if len(circles):
        (x, y), radius = circles[-1]
        #print(radius)
        if float(radius) > 1.85:
            return (int(x), int(y)), int(radius)

def find_basket(src):
    contours_basket, _ = cv2.findContours(src, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    for contour in contours_basket:
        M = cv2.moments(contour)

        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        (x, y, w, h) = cv2.boundingRect(contour)
        #print("Basket size: ", w*h)
        #print("wh " + str(w*h))
        if  h >= 15 and (w*h >= 1600):
            return (int(x), int(y)), (int(w), int(h)), (int(cX), int(cY))


#
# def line_intersection(line1, line2):
#     for x in line1:
#         for y in x:
#             if y < 0:
#                 line1[line1[x].index(y)] = 0
#     for x in line2:
#         for y in x:
#             if y < 0:
#                 line2[line2[x].index(y)] = 0
#     print(line1, line2)
#     xdiff = [line1[0][0] - line1[1][0], line2[0][0] - line2[1][0]]
#     ydiff = [line1[1][1] - line1[0][1], line2[1][1] - line2[0][1]]
#
#     def det(a, b):
#         return a[0] * b[1] - a[1] * b[0]
#
#     div = det(xdiff, ydiff)
#     #print("DIV TIME " + str(div))
#     print(ydiff, xdiff)
#     #print(div)
#     if div == 0:
#         print("DIV IS 0")
#         return False
#     else:
#         return True

def intersectionFinder(line1, line2):

    line1 = LineString([(line1[0][0], line1[0][1]), (line1[1][0], line1[1][1])])
    line2 = LineString([(line2[0][0], line2[0][1]), (line2[1][0], line2[1][1])])

    result = line1.intersection(line2)
    #print(result)
    if str(result) == "GEOMETRYCOLLECTION EMPTY":
        return False
    else:
        return True