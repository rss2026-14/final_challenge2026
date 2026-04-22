import numpy as np
import cv2


img=cv2.imread('../racetrack_images/lane_3/image35.png')
h, w = img.shape[:2]

# Create black mask
mask = np.zeros((h, w), dtype=np.uint8)
top = int(h * 0.4)
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
eroded = cv2.erode(track_mask, kernel, iterations=2)
dilated = cv2.dilate(eroded, kernel, iterations=2)
processed = cv2.bitwise_and(hsv, hsv, mask=dilated)

# Find edges
gray = cv2.cvtColor(processed, cv2.COLOR_RGB2GRAY)
edges = cv2.Canny(gray, 120, 180)
lines = cv2.HoughLinesP(edges, 1, np.pi/180, 68, minLineLength=160, maxLineGap=20)
for line in lines:
    x1, y1, x2, y2 = line[0]
    cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 3)

processed = cv2.cvtColor(processed, cv2.COLOR_HSV2RGB)
cv2.imshow("Image", img)
cv2.imshow("Masked", masked)
cv2.imshow("Color Masked", processed)
cv2.waitKey(0)
cv2.destroyAllWindows()
