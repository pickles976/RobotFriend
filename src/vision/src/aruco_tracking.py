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

marker_dict = None

def loadCameraMatrix():
    with open(os.path.join(path, "camera_matrix.json"), 'r') as f:
        data = json.load(f)
        camera_matrix = np.array(data["camera_matrix"], dtype=np.float32)
        dist_coeffs = np.array(data["dist_coeff"][0], dtype=np.float32)
        print("Camera Matrix: %s"%camera_matrix)
        print("Distortion Coefficients: %s"%dist_coeffs)

def loadArucoCoordinates():
    with open(os.path.join(path, "aruco_markers.json"), 'r') as f:
        dict = json.load(f)

        # convert keys to numbers
        dict = {int(k):int(v) for k,v in dict.items()}

        # convert to np array
        for key in dict:
            dict[key] = np.array(dict[key], dtype=np.float32)

        print("Loaded markers")
        print(dict)

        return dict

def findArucoMarkers(img, markerSize = 5, totalMarkers=250, draw=True):    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    key = cv2.aruco.DICT_ARUCO_ORIGINAL
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
    if draw:
        aruco.drawDetectedMarkers(img, bboxs)
    return [bboxs, ids]

loadCameraMatrix()
marker_dict = loadArucoCoordinates()

path = os.path.join(path, "localization_images/")
imName= "2_4_0.jpg"

img = cv2.imread(path+imName)

arucofound = findArucoMarkers(img, markerSize = 5)

points = arucofound[0]
ids = arucofound[1]

for i in range(0, len(points)):
    points_3D = marker_dict[ids[i][0]]
    points_2D = points[i]
    print(points_3D)
    success, rotation_vector, translation_vector = cv2.solvePnP(points_3D, points_2D, camera_matrix, dist_coeffs, flags=0)
    print("Rotation Vector: %s"%rotation_vector)
    print("Translation Vector: %s"%translation_vector)

# cv2.imshow('img',img)
# cv2.waitKey(0)