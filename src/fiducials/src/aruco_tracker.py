#!/usr/bin/python
# https://pyimagesearch.com/2020/12/28/determining-aruco-marker-type-with-opencv-and-python/
# SOLVEPNP
# https://learnopencv.com/head-pose-estimation-using-opencv-and-dlib/
import cv2
import cv2.aruco as aruco
import numpy as np
import json
import os
from numpy.linalg import inv

class ArucoTracker:

    def __init__(self, camera_matrix_json, aruco_marker_json):
        self.camera_matrix, self.dist_coeffs = self._loadCameraMatrix(camera_matrix_json)
        self.marker_dict = self._loadArucoCoordinates(aruco_marker_json)

    def _loadCameraMatrix(self, camera_matrix_json):
        with open(camera_matrix_json, 'r') as f:
            data = json.load(f)
            camera_matrix = np.array(data["camera_matrix"], dtype=np.float32)
            dist_coeffs = np.array(data["dist_coeff"], dtype=np.float32).transpose()
            print("Camera Matrix: %s"%camera_matrix)
            print("Distortion Coefficients: %s"%dist_coeffs)
            return (camera_matrix, dist_coeffs)

    def _loadArucoCoordinates(self, aruco_markers_json):
        with open(aruco_markers_json, 'r') as f:
            dict = json.load(f)

            # convert keys to numbers
            dict = {int(k):v for k,v in dict.items()}

            # convert to np array
            for key in dict:
                dict[key] = np.array(dict[key], dtype=np.float32)

            print("Loaded markers")

            return dict

    def _findArucoMarkers(self, img, markerSize = 5, totalMarkers=250, draw=True):    
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        key = cv2.aruco.DICT_ARUCO_ORIGINAL
        arucoDict = aruco.Dictionary_get(key)
        arucoParam = aruco.DetectorParameters_create()
        bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
        if draw:
            aruco.drawDetectedMarkers(img, bboxs)
        return [bboxs, ids]

    def getPoseEstimatesFromImage(self, img):

        arucofound = self._findArucoMarkers(img, markerSize = 5)

        print("Found %s markers"%len(arucofound[0]))

        points = arucofound[0]
        ids = arucofound[1]

        pose_estimates = []

        for i in range(0, len(points)):

            points_3D = self.marker_dict[ids[i][0]]
            points_2D = points[i][0]

            success, rotation_vector, translation_vector = cv2.solvePnP(points_3D, points_2D, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE)

            if success:

                # convert to homogeneous transform matrix
                rmat, _ = cv2.Rodrigues(rotation_vector) # rotation vector to rotation matrix
                transform = np.eye(4, dtype=np.float32)
                transform[:3, :3] = rmat
                transform[:3, 3] = translation_vector.reshape(3)

                # compute the inverse to get absolute world position
                transform = inv(transform)

                # convert to ft
                translation = transform[:3,3]
                translation = list(map(lambda x: x / 304.80, translation))
                print(translation)

                # print rotation
                rotation = transform[:3,:3]
                print(rotation)

                pose_estimates.append(transform)

        return pose_estimates
            
