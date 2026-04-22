import numpy as np
import cv2

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
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
        bbox: ((x1, y1), (x2, y2)); the bounding box of the track, unit in px
            (x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
    """
    ########## YOUR CODE STARTS HERE ##########
    h, w = img.shape[:2]

    # Create black mask
    mask = np.zeros((h, w), dtype=np.uint8)

    # Define the region you want to keep
    top = int(h * 0.5)
    bottom = int(h * 0.9)
    left = int(h * 0.2)
    right = int(h * 0.8)

    mask[top:bottom, left:right] = 255

    # Apply mask
    masked = cv2.bitwise_and(img, img, mask=mask)

    # Apply color mask
    HSV_img = cv2.cvtColor(masked, cv2.COLOR_BGR2HSV)

    lower_bound = np.array([0, 0, 130])
    upper_bound = np.array([180, 50, 255])

    track_mask = cv2.inRange(HSV_img, lower_bound, upper_bound)

    # Erode & dilate
    kernel = np.ones((3, 3), np.uint8)
    eroded = cv2.erode(track_mask, kernel, iterations=2)
    dilated = cv2.dilate(eroded, kernel, iterations=2)

    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x1, y1, w, h = cv2.boundingRect(largest_contour)
        return ((x1, y1), (x1 + w, y1 + h))

    return None  # No valid cone found
