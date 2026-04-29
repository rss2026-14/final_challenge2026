import numpy as np
import cv2
import math as m
from sympy import Point, Line
img=cv2.imread('../../racetrack_images/lane_3/image16.png')
h, w = img.shape[:2]

# Create black mask
mask = np.zeros((h, w), dtype=np.uint8)
top = int(h * 0.45)
bottom = int(h * 0.95)
left = int(w * 0.05)
right = int(w * 0.95)
mask[top:bottom, left:right] = 255
masked = cv2.bitwise_and(img, img, mask=mask)

# Apply color mask
hsv = cv2.cvtColor(masked, cv2.COLOR_RGB2HSV)
lower_bound = np.array([0, 0, 130])
upper_bound = np.array([180, 50, 255])
track_mask = cv2.inRange(hsv, lower_bound, upper_bound)
kernel = np.ones((3, 3), np.uint8)
eroded = cv2.erode(track_mask, kernel, iterations=1)
dilated = cv2.dilate(eroded, kernel, iterations=2)
colormasked = cv2.bitwise_and(hsv, hsv, mask=dilated)

# Find edges
gray = cv2.cvtColor(colormasked, cv2.COLOR_RGB2GRAY)
edges = cv2.Canny(gray, 100, 180)
lines = cv2.HoughLinesP(edges, 1, np.pi/180, 68, minLineLength=80, maxLineGap=20)

# only save lines with angle > 15 degrees
Xs=[]
linemask=[]
if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        angle=m.atan(abs((y2-y1)/(x2-x1)))
        if angle>(15*m.pi/180):
            linemask.append(line)
            Xs.append(x1)
            Xs.append(x2)

# find left and right lanes
if linemask is not None:
    left=None
    right=None
    lanes=[]
    for line in linemask:
        x1, y1, x2, y2 = line[0]
        if (x1==min(Xs) or x2==min(Xs)) and left is None:
            left=Line(Point(x1,y1),Point(x2,y2))
            lanes.append(line)
            cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 3)
        if (x1==max(Xs) or x2==max(Xs)) and right is None:
            right=Line(Point(x1,y1),Point(x2,y2))
            lanes.append(line)
            cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 3)

    intersection=left.intersection(right)
    if intersection:
        print(intersection[0].evalf())
        px,py=intersection[0].evalf()
        cv2.circle(img, (int(px), int(py)), 5, (0, 0, 255), -1)

colormasked = cv2.cvtColor(colormasked, cv2.COLOR_HSV2RGB)
cv2.imshow("Image", img)
#cv2.imshow("Masked", masked)
#cv2.imshow("Color Masked", colormasked)
cv2.waitKey(0)
cv2.destroyAllWindows()
