import numpy as np
import cv2
import math as m
from sympy import Point, Line
#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v
#  v
#  v
#  v
###############################################################

def cd_color_segmentation(img):
    """
    Implement track detection using color segmentation algorithm
    Input:
        img: np.3darray; the input image. BGR.
    Return:
        pixel: (px,py); pixel of track to drive to, unit in px
        linemask: list of lines, each with ((x1,y1),(x2,y2))
    """
    ########## YOUR CODE STARTS HERE ##########
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
    hsv = cv2.cvtColor(masked, cv2.COLOR_BGR2HSV)
    lower_bound = np.array([0, 0, 130])
    upper_bound = np.array([180, 50, 255])
    track_mask = cv2.inRange(hsv, lower_bound, upper_bound)
    kernel = np.ones((3, 3), np.uint8)
    eroded = cv2.erode(track_mask, kernel, iterations=1)
    dilated = cv2.dilate(eroded, kernel, iterations=2)
    colormasked = cv2.bitwise_and(hsv, hsv, mask=dilated)

    # Find edges
    gray = cv2.cvtColor(colormasked, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 100, 180)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 68, minLineLength=80, maxLineGap=20)

    # only save lines with angle > 15 degrees
    Xs=[]
    linemask=[]
    if lines is None:
        return None
    for line in lines:
        x1, y1, x2, y2 = line[0]
        angle=m.atan(abs((y2-y1)/(x2-x1)))
        if angle>(15*m.pi/180):
            linemask.append(line)
            Xs.append(x1)
            Xs.append(x2)

    # find left and right lanes
    if linemask is None:
        return None
    left=None
    right=None
    lanes=[]
    for line in linemask:
        x1, y1, x2, y2 = line[0]
        if (x1==min(Xs) or x2==min(Xs)) and left is None:
            left=Line(Point(x1,y1),Point(x2,y2))
            lanes.append(line)
        if (x1==max(Xs) or x2==max(Xs)) and right is None:
            right=Line(Point(x1,y1),Point(x2,y2))
            lanes.append(line)

    if left is not None and right is not None:
        intersection=left.intersection(right)
        if intersection:
            px,py=intersection[0].evalf()
            return px,py,lanes
    return None
