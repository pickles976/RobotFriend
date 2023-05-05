#!/usr/bin/python
### Tests different solver methods against the aruco fiducial images
from aruco_tracker import ArucoTracker
import cv2
import os
import numpy as np

path = "./src/fiducials/util/"
image_folder = "localization_images_1280_video"
impath = os.path.join(path, image_folder)
images = os.listdir(impath)

tracker = ArucoTracker("src/fiducials/util/camera_matrix.json","src/fiducials/util/aruco_markers.json")
methods = {
    "SOLVEPNP_ITERATIVE" : cv2.SOLVEPNP_ITERATIVE,

}

scores = {}

for key in methods:

    # for each solver method
    sumError = 0

    # for each image in the folder
    for imName in images:

        actual = imName.split(".")[0].replace(",", ".").split("_")
        actual = list(map(lambda x: float(x), actual))
        actual = np.array(actual, dtype=np.float32)
        actual[2] = 0.21

        img = cv2.imread(os.path.join(impath, imName))
        poses = tracker.getPoseEstimatesFromImage(img, methods[key])

        print("Actual: %s"%actual)
        for pose in poses:
            trans = pose[:3,3]
            trans = list(map(lambda x: x / 304.80, trans))
            trans = np.array(trans, dtype=np.float32)
            print("Estimate: %s"%trans)

            error = np.linalg.norm(np.subtract(actual, trans))
            error = error ** 2
            sumError += error

    mse = sumError / len(images)
    print(mse)
    scores[key] = mse

print(scores)
