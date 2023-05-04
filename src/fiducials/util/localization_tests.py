#!/usr/bin/python
### Tests different solver methods against the aruco fiducial images
from aruco_tracker import ArucoTracker
import cv2
import os
import numpy as np

path = "./src/fiducials/util/"
image_folder = "localization_images"
images = os.listdir(os.path.join(path, image_folder))

tracker = ArucoTracker("src/fiducials/util/camera_matrix.json","src/fiducials/util/aruco_markers.json")
methods = {
    "SOLVEPNP_ITERATIVE" : cv2.SOLVEPNP_ITERATIVE,
    "SOLVEPNP_P3P" : cv2.SOLVEPNP_P3P,
    "SOLVEPNP_AP3P" : cv2.SOLVEPNP_AP3P,
    "SOLVEPNP_EPNP" : cv2.SOLVEPNP_EPNP,
    "SOLVEPNP_IPPE" : cv2.SOLVEPNP_IPPE,
    "SOLVEPNP_IPPE_SQUARE" : cv2.SOLVEPNP_IPPE_SQUARE
}

scores = {}

for key in methods:

    # for each solver method
    sumError = 0

    # for each image in the folder
    for imName in images:

        actual = imName.split(".")[0].replace("-", ".").split("_")
        actual = list(map(lambda x: float(x), actual))
        actual = np.array(actual, dtype=np.float32)

        img = cv2.imread(os.path.join(path, "localization_images/")+imName)
        poses = tracker.getPoseEstimatesFromImage(img, methods[key])

        for pose in poses:
            trans = pose[:3,3]
            trans = list(map(lambda x: x / 304.80, trans))
            trans = np.array(trans, dtype=np.float32)

            error = np.linalg.norm(np.subtract(actual, trans))
            error = error ** 2
            sumError += error

    mse = sumError / len(images)
    print(mse)
    scores[key] = mse

print(scores)
