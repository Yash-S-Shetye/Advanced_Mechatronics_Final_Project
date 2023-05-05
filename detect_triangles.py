# import required libraries
import cv2
import numpy as np

# read the input image
img = cv2.imread('triangle_left.jpg')

# convert the image to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

moment = cv2.moments(gray)

X = int(moment ["m10"] / moment["m00"])
Y = int(moment ["m01"] / moment["m00"])

# apply thresholding to convert the grayscale image to a binary image
ret,thresh = cv2.threshold(gray,50,255,0)

# find the contours
contours,hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
print("Number of contours detected:",len(contours))

for cnt in contours:
   approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
   if len(approx) == 3:
      img = cv2.drawContours(img, [cnt], -1, (0,255,255), 3)

      # compute the center of mass of the triangle
      M = cv2.moments(cnt)
      if M['m00'] != 0.0:
        x = int(M['m10']/M['m00'])
        y = int(M['m01']/M['m00'])

      if x > X:
         print("Left")
      else:
         print("Right")

      cv2.putText(img, 'Triangle', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

cv2.imshow("Shapes", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
