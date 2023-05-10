#! /usr/bin/env python

import cv2
import numpy as np
import serial
from cv2 import aruco

def detect_triangles():

    img = cv2.VideoCapture(0)

    result, frame = img.read()

    if result:
        cv2.imwrite("Image.jpg", frame)

    cv2.destroyAllWindows()

    image = cv2.imread('Image.jpg')

    # Convert the frame to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    moment = cv2.moments(gray)

    X = int(moment["m10"] / moment["m00"])
    Y = int(moment["m01"] / moment["m00"])

    # apply thresholding to convert the grayscale image to a binary image
    ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    # find the contours
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    print("Number of contours detected:", len(contours))
    # Filter contours by area
    contours = [c for c in contours if cv2.contourArea(c) > 100]

    triangle = None
    for cnt in contours:
        triangle = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
        print(triangle[0][0],triangle[1][0],triangle[2][0])
        if len(triangle) == 3:
            image = cv2.drawContours(image, [cnt], 0, (0, 255, 0), 3)

            # compute the center of mass of the triangle
            M = cv2.moments(cnt)
            if M['m00'] != 0.0:
                x = int(M['m10'] / M['m00'])
                y = int(M['m01'] / M['m00'])
                count_left = 0
                print(x,y)
                for c in triangle:
                    count_left += 1 if c[0][0] < x else 0
                    if count_left < 1.5:
                        print('Left')
                        message = 'l'
                        ser.write(message.encode())
                    else:
                        print('Right')
                        message = 'r'
                        ser.write(message.encode())
        else:
            print('No triangle detected')


    cv2.imshow('Image.jpg', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def detect_aruco_tags():
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
            if id%2==0 :
                print("Not Defective")
                tags = 'n'
                ser.write(tags.encode())
            else:
                print("Defective")
                tags = 'd'
                ser.write(tags.encode())
    else:
        print("No ArUco markers found in the image.")


if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600)

    while True:
        data = ser.readline()
        print(data)
        if data == 1:
            detect_triangles()
    
        elif data == 2:
            detect_aruco_tags()



