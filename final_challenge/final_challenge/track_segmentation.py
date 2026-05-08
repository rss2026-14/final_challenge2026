import numpy as np
import cv2
import math as m

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
###############################################################

def extend_line(line,h):
    """
    Given a Hough line [[x1, y1, x2, y2]], return a line spanning the image height
    """
    x1, y1, x2, y2 = line[0]

    m=(y2-y1)/(x2-x1)
    x0=-y1/m+x1
    x3=(h-y1)/m+x1

    return [x0, 0.0, x3, h]

def line_x_at_y(line, y_query):
    """
    Given a Hough line [[x1, y1, x2, y2]], return the x position
    where the line crosses y = y_query.
    """
    x1, y1, x2, y2 = line

    if abs(y2 - y1) < 1e-6:
        return None

    t = (y_query - y1) / float(y2 - y1)

    # Allow small extrapolation, but reject lines that are too far away
    if t < -0.5 or t > 1.5:
        return None

    return x1 + t * (x2 - x1)


def cd_color_segmentation(img):
    """
    Detects lane boundaries and returns a target point near the lane center.

    Input:
        img: np.ndarray, BGR image

    Return:
        x_target, y_target, selected_lines
    """

    h, w = img.shape[:2]

    # Focus on the lower part of the image where the track is closest
    roi_mask = np.zeros((h, w), dtype=np.uint8)

    # Trapezoid ROI: removes lots of irrelevant upper-image detections
    polygon = np.array([[
        (int(0.0 * w), int(0.95 * h)),
        (int(0.0 * w), int(0.45 * h)),
        (int(1.0 * w), int(0.45 * h)),
        (int(1.0 * w), int(0.95 * h)),
    ]], dtype=np.int32)

    cv2.fillPoly(roi_mask, polygon, 255)
    masked_img = cv2.bitwise_and(img, img, mask=roi_mask)

    # White mask in HSV
    hsv = cv2.cvtColor(masked_img, cv2.COLOR_BGR2HSV)

    # White line threshold
    lower_white = np.array([0, 0, 130])
    upper_white = np.array([180, 65, 255])
    white_mask = cv2.inRange(hsv, lower_white, upper_white)

    kernel = np.ones((5, 5), np.uint8)
    white_mask = cv2.erode(white_mask, kernel, iterations=1)
    white_mask = cv2.dilate(white_mask, kernel, iterations=2)

    edges = cv2.Canny(white_mask, 80, 160)

    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180,
        threshold=45,
        minLineLength=30,
        maxLineGap=25
    )

    if lines is None:
        return None

    left_candidates = []
    right_candidates = []
    debug_lines = []

    image_center_x = w / 2.0

    # Pick a row to aim at. Larger y is closer to the robot.
    y_target = int(0.4 * h)

    for line in lines:
        x1, y1, x2, y2 = line[0]
        x0,y0,x3,y3=extend_line(line,h)
        dx = x2 - x1
        dy = y1 - y2

        if abs(dx) < 1e-6:
            continue

        slope = dy / float(dx)
        angle = abs(np.degrees(np.arctan2(dy, dx)))

        # Reject nearly horizontal lines.
        # These are usually crosswalk/intersection/stop-line-like markings.
        if angle < 10:
            continue

        # Reject almost vertical tiny artifacts if needed.
        if angle > 85:
            continue

        x_at_target = line_x_at_y([x0,y0,x3,y3], y_target)

        if x_at_target is None:
            continue

        # Reject lines that cross too close to image center.
        # This prevents the center white line from becoming the target.


        debug_lines.append(line)

        # In image coordinates, left lane usually has negative slope.
        # Right lane usually has positive slope.
        if slope > 0 and x_at_target < image_center_x:
            left_candidates.append((line, x_at_target))
        elif slope < 0 and x_at_target > image_center_x:
            right_candidates.append((line, x_at_target))

    if len(left_candidates) == 0 and len(right_candidates) == 0:
        print("both empty")
        return None

    selected_lines = []

    left_x = None
    right_x = None

    if len(left_candidates) > 0:
        # Choose the left line closest to the center, not the extreme outside line.
        left_line, left_x = max(left_candidates, key=lambda item: item[1])
        selected_lines.append(left_line)
        print("left is not empty")

    if len(right_candidates) > 0:
        # Choose the right line closest to the center, not the extreme outside line.
        right_line, right_x = min(right_candidates, key=lambda item: item[1])
        selected_lines.append(right_line)
        print("right is not empty")

    # If both lane boundaries are visible, drive between them.
    if left_x is not None and right_x is not None:
        x_target = int((left_x + right_x) / 2.0)
        print("both not empty")

    # If only left boundary is visible, offset to the right by an estimated half lane width.
    elif left_x is not None:
        estimated_lane_half_width_px = int(0.12 * w)
        x_target = int(left_x + estimated_lane_half_width_px)

    # If only right boundary is visible, offset to the left by an estimated half lane width.
    elif right_x is not None:
        estimated_lane_half_width_px = int(0.12 * w)
        x_target = int(right_x - estimated_lane_half_width_px)

    else:
        return None

    # Clamp target inside image
    x_target = int(np.clip(x_target, 0, w - 1))
    y_target = int(np.clip(y_target, 0, h - 1))

    return x_target, y_target, selected_lines
