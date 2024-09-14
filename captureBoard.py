import numpy as np
import cv2

cam = cv2.VideoCapture(2)

if not cam.isOpened():
    for i in range(0, 11):
        cam = cv2.VideoCapture(i)
        if cam.isOpened():
            break

if not cam.isOpened():
    print("Can't open camera")
    exit()

T = 75

while True:
    
    _, image = cam.read()
    image = image[:-25]

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
#    mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 7)
    _, mask = cv2.threshold(gray, T, 255, cv2.THRESH_BINARY)
#    mask = cv2.medianBlur(mask, 11)
#    mask = cv2.GaussianBlur(mask, (11, 11), 0)

    edges = cv2.Canny(gray, 50, 100)
    cnts, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image, cnts, 0, (0, 255, 0), 2)

    cv2.imshow("Frame", image)
    cv2.imshow("Gray", gray)
    cv2.imshow("Thresh", mask)
    cv2.imshow("Edged", edges)

    if cv2.waitKey(1) == ord('q'):
        break

cam.release()
