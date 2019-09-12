import cv2

cap = cv2.VideoCapture(1)
bgr = None
paused = False

while cap.isOpened():
    # Read frame in BGR
    if not paused or bgr is None:
        _, bgr = cap.read()

    cv2.imshow("bgr", bgr)

    # Convert to HSV (H: 0-180, S: 0-255, V: 0-255)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv", hsv)

    # Apply color mask
    mask = cv2.inRange(hsv, (0, 0, 100), (40, 100, 255))
    cv2.imshow("mask", mask)

    # Keyboard input
    key = cv2.waitKey(10)

    # Toggle pause
    if key & 0xFF == ord("p"):
        paused = not paused

    # Select region
    if key & 0xFF == ord("s"):
        x0, y0, x1, y1 = cv2.selectROI("roi", bgr)
        x, y = (x0 + x1) // 2, (y0 + y1) // 2

        print("BGR:", bgr[y, x])
        print("HSV:", hsv[y, x])

        cv2.destroyWindow("roi")
        paused = True

    # Exit
    if key & 0xFF == ord("q"):
        break

cap.release()
