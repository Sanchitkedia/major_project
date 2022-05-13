#!/usr/bin/env python

import cv2
import numpy as np
import keyboard
import math

cap = cv2.VideoCapture(0)

while True:
    _, frame = cap.read()
    if frame is None:
        break

    width  = cap.get(3)
    height = cap.get(4)
    frame=cv2.circle(frame,(60,240),5,(255,0,0),-1)
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    _,mask = cv2.threshold(gray, 150,255,cv2.THRESH_BINARY)
    edges = cv2.Canny(gray, 50, 200)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180,170, minLineLength=10, maxLineGap=100)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 1)
    cv2.imshow("frame",frame)
    cv2.imshow("mask",mask)

    k = cv2.waitKey(30) & 0xFF
    img_counter = 0
    if keyboard.is_pressed("space"):
        img_name = "Hough{}.png".format(img_counter)
        cv2.imwrite(img_name, frame)
        img_counter += 1
        img_name = "Mask{}.png".format(img_counter)
        cv2.imwrite(img_name, mask)
        img_counter += 1
        break


cap.release()
cv2.destroyAllWindows()

img1 = cv2.imread("Hough0.png")
img2 = cv2.imread("Mask1.png")
x = 0
y = 0

blue = [255,0,0]

X,Y = np.where(np.all(img1==blue,axis=2))
zipped = np.column_stack((X,Y))
x_d = (((2*Y)/2)-60) * 0.06
y_d = (((2*X)/2)-240) * 0.075

FinalCoordinates = np.array([x_d,y_d])
np.savetxt("xD.csv",FinalCoordinates,delimiter=",")
print(FinalCoordinates)
print(zipped)

for x in zipped:
    img2[X,Y] = [255,0,0]
cv2.imwrite("Final.png",img2)