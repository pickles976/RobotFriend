from gettext import translation
import json
import os
from turtle import dot
import numpy as np
from math import sin, cos
from scipy.spatial.transform import Rotation   

path = "./src/vision/src/"
marker_file = "aruco_marker_layout.json"

aruco_markers = None

with open(os.path.join(path, marker_file), 'r') as f:

    dict = json.load(f)

    # get translations from dict
    trans = {}
    for key in dict:
        trans[key] = dict[key]["translation"]

    # convert to mm
    for key in trans:
        trans[key] = list(map(lambda x: x * 304.80,trans[key]))

    # convert to np ndarray
    for key in trans:
        trans[key] = np.array(trans[key], dtype=np.float32)

    # calculate four corners from 80cm width
    for key in trans:
        corners = [trans[key]]
        corners.append(np.add(trans[key], np.array([80, 0, 0], np.float32)))
        corners.append(np.add(trans[key], np.array([80, 0, -80], np.float32)))
        corners.append(np.add(trans[key], np.array([0, 0, -80], np.float32)))
        trans[key] = corners
    
    # rotate corners by rotation matrix
    for key in dict:

        angles = dict[key]["rotation"]
        r = Rotation.from_euler("xyz",angles,degrees=True)
        new_rotation_matrix = r.as_matrix()

        corners = trans[key]
        
        for i in range(0, len(corners)):
            corners[i] = corners[i].dot(new_rotation_matrix).tolist()
            
        trans[key] = corners

    # save to new dict
    aruco_markers = trans

# Save to json
with open(os.path.join(path, "aruco_markers.json"), "w") as outfile:
    json_object = json.dumps(aruco_markers, indent=4)
    outfile.write(json_object)