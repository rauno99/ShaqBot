import movement
import cv2

def moveKeyboard(key, ser):
    cap = cv2.VideoCapture(2)
    while True:
        _, frame = cap.read()
        cv2.imshow("frame", frame)
        print("keyboard")
        if key == ord("w"):
            movement.moveForward(ser)