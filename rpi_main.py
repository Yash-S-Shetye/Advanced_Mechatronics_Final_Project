#! /usr/bin/env python

# Importing all the required libraries
import serial
import time
import cv2
import numpy as np
from cv2 import aruco
from picamera2 import Picamera2


# Load the ArUco dictionary and parameters
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
parameters = aruco.DetectorParameters()
parameters.adaptiveThreshWinSizeMax = 100

# Start the serial communication between raspberry pi and arduino
ser = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(2)

# Define parameters for raspberry pi camera
picam2 = Picamera2()
picam2.resolution = (1280, 720)
picam2.framerate = 10
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (1280, 720)}))
picam2.start() # Start raspberry pi camera

# Function for detecting triangles for direction
def detect_triangles(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define lower and upper bounds of blue color in HSV
    lower_blue = np.array([90, 50, 50])
    upper_blue = np.array([130, 255, 255])

    # Create a mask of blue pixels
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Perform morphological operations
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) > 100]

    for contour in contours:
        triangle = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
        if len(triangle) == 3:
            cv2.drawContours(img, [triangle], 0, (0, 255, 0), 2)
            # compute the center of mass of the triangle
            M = cv2.moments(contour)
            if M['m00'] != 0.0:
                x = int(M['m10'] / M['m00'])
                y = int(M['m01'] / M['m00'])
                count_left = 0
                for c in triangle:
                    count_left += 1 if c[0][0] < x else 0
                if count_left < 1.5:
                    ser.write(b'l')  # Send message to arduino
                    print("Left")
                    return True

                else:
                    ser.write(b'r') # Send message to arduino
                    print("Right")
                    return True
    print("No Triangle")
    return False
# Function for detecting aruco tags
def detect_aruco(img):
    # Convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Detect the ArUco markers in the image
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # If any markers are detected, print their IDs
    if ids is not None:
        for i, id in enumerate(ids):
            print(f"ArUco marker {id}:")
            #print(f"  Corners: {corners[i]}")
            if id%2==0 :
               ser.write(b'n') # Send message to arduino
               print("Not Defective")

            else:
                ser.write(b'd') # Send message to arduino
                print("Defective")
        return True

    else:
        #print("No ArUco markers found in the image.")
        return False

# Function for final no man's land motion
def final_part(img):
    # Convert image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Detect ArUco markers
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:

        # Calculate centroids
        centroids = []
        for i in range(len(ids)):
            # Calculate centroid by averaging the corner coordinates
            centroid = np.mean(corners[i][0], axis=0)
            centroid = centroid.astype(int)
            centroids.append(centroid)

            print(centroid)
            ratio = centroid[0]/640
            if ratio >= 0.9 and ratio <= 1.1:
                ser.write(b'g') # Send message to arduino
                print("Final")
                return True
            else:
                print("tag")
                return False
    return False
# Main loop
while True:
      img = picam2.capture_array() # Convert video feed to numpy array for processing

      # Wait for message from arduino 
      if ser.in_waiting > 0:
         data = ser.read().decode() # Read message from arduino
         print(data)
         if data == 't':
            # Go to triangles detection function
            while not detect_triangles(img):
                  time.sleep(0.2)
                  img = picam2.capture_array()

         elif data == 'a':
            # Start time to send multiple aruco tags detected
            start_millis = time.time() * 1000
            # Go to aruco tags detection function
            while not detect_aruco(img):
                  stop_millis = time.time() * 1000 # End time once all aruco tags are detected
                  # COme out of loop after 2 seconds
                  if stop_millis - start_millis > 2000: 
                     break

                  time.sleep(0.2)
                  img = picam2.capture_array()


         elif data == 'f':
            # Go to final no man's land detection function
            while not final_part(img):
                  time.sleep(0.1)
                  img = picam2.capture_array()

ser.close() # Close communication





