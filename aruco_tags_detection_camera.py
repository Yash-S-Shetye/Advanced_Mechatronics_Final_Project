#! /usr/bin/env python

import cv2
from cv2 import aruco

# Load the ArUco dictionary and parameters
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
parameters = aruco.DetectorParameters()

image = cv2.VideoCapture(0)

result, frame = image.read()

if result:
   
   cv2.imwrite("aruco_test.jpg", frame)

   cv2.destroyAllWindows()

# Load the image
img = cv2.imread('aruco_test.jpg')

# Convert the image to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Detect the ArUco markers in the image
corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

# If any markers are detected, print their IDs
if ids is not None:
    for i, id in enumerate(ids):
        print(f"ArUco marker {id}:")
        print(f"  Corners: {corners[i]}")
else:
    print("No ArUco markers found in the image.")
