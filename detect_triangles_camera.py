# import required libraries
import cv2
import numpy as np

# read the input image
img = cv2.VideoCapture(0)

result, frame = img.read()

if result:
   
   cv2.imwrite("Image.jpg", frame)

   cv2.destroyAllWindows()

image = cv2.imread('Image.jpg')
    
# Convert the frame to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

moment = cv2.moments(gray)

X = int(moment ["m10"] / moment["m00"])
Y = int(moment ["m01"] / moment["m00"])

# apply thresholding to convert the grayscale image to a binary image
ret,thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

# find the contours
contours,hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
print("Number of contours detected:",len(contours))

for cnt in contours:
   approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
   if len(approx) == 3:
      image = cv2.drawContours(image, [cnt], 0, (0, 255, 0), 3)

      # compute the center of mass of the triangle
      M = cv2.moments(cnt)
      if M['m00'] != 0.0:
         x = int(M['m10']/M['m00'])
         y = int(M['m01']/M['m00'])

      if x > X:
         print("Left")
      else:
         print("Right")
    
cv2.imshow('Image.jpg', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
