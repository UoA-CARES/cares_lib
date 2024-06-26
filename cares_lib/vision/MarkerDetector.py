import cv2
import math
import cares_lib.utils.utils as utils
from abc import ABC

class MarkerDetector(ABC):
    """Base class for marker detectors like ArucoDetector and STagDetector"""
    def __init__(self, marker_size):
        self.marker_size = marker_size

    def get_orientation(self, r_vec):
        r_matrix, _ = cv2.Rodrigues(r_vec)
        roll, pitch, yaw = utils.rotation_to_euler(r_matrix)

        def validate_angle(degrees):
            return degrees % 360

        roll  = validate_angle(math.degrees(roll))
        pitch = validate_angle(math.degrees(pitch))
        yaw   = validate_angle(math.degrees(yaw))

        return [roll, pitch, yaw]

    def get_pose(self, t_vec, r_vec):
        pose = {}
        pose["position"] = t_vec[0]
        pose["orientation"] = self.get_orientation(r_vec)
        pose["r_vec"] = r_vec
        return pose
        
    def get_marker_poses(self, image, camera_matrix, camera_distortion, corners, ids, display=True):
        marker_poses = {}

        if len(corners) > 0:
            r_vecs, t_vecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, camera_matrix, camera_distortion)

            image_copy = image.copy()
            
            cv2.aruco.drawDetectedMarkers(
                image_copy, corners, ids, borderColor=(0, 0, 255))

            for i in range(0, len(r_vecs)):
                cv2.drawFrameAxes(image_copy, camera_matrix,
                               camera_distortion, r_vecs[i], t_vecs[i], self.marker_size/2.0, 3)

            if display:
                cv2.imshow("Frame", image_copy)
                cv2.waitKey(100)

            for i in range(0, len(r_vecs)):
                id = ids[i][0]
                r_vec = r_vecs[i]
                t_vec = t_vecs[i]
          	    #TODO: change this to output something less bulky than two arrays 
                marker_poses[id] = self.get_pose(t_vec, r_vec)

        return marker_poses
