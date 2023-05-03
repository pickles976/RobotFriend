#!/usr/bin/python
# https://pyimagesearch.com/2020/12/28/determining-aruco-marker-type-with-opencv-and-python/

import cv2
import cv2.aruco as aruco
import numpy as np
import json
import os

path = "./src/vision/src/"

camera_matrix = None
dist_coeffs = None

def loadCameraMatrix():
    with open(os.path.join(path, "camera_matrix.json"), 'r') as f:
        data = json.load(f)
        camera_matrix = np.array(data["camera_matrix"], dtype=np.float32)
        dist_coeffs = np.array(data["dist_coeff"][0], dtype=np.float32)
        print("Camera Matrix: %s"%camera_matrix)
        print("Distortion Coefficients: %s"%dist_coeffs)

def findArucoMarkers(img, markerSize = 5, totalMarkers=250, draw=True):    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    key = cv2.aruco.DICT_ARUCO_ORIGINAL
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
    print(ids)
    if draw:
        aruco.drawDetectedMarkers(img, bboxs)
    return [bboxs, ids]

loadCameraMatrix()

path = os.path.join(path, "localization_images/")
imName= "2_4_0.jpg"

img = cv2.imread(path+imName)

arucofound = findArucoMarkers(img, markerSize = 5)

cv2.imshow('img',img)
cv2.waitKey(0)