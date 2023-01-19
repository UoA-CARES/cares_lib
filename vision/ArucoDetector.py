import cv2
import math
import numpy as np


class ArucoDetector:
    def __init__(self, marker_size, dictionary_id=cv2.aruco.DICT_4X4_50) -> None:
        self.dictionary = cv2.aruco.Dictionary_get(
            dictionary_id)  # aruco dictionary
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.marker_size = marker_size  # mm

    def is_close(self, x, y, rtol=1.e-5, atol=1.e-8):
        # this is a tolerance thingy i think
        return abs(x - y) <= atol + rtol * abs(y)

    def calculate_euler_angles(self, R):
        phi = 0.0
        if self.is_close(R[2, 0], -1.0):
            theta = math.pi / 2.0
            psi = math.atan2(R[0, 1], R[0, 2])
        elif self.is_close(R[2, 0], 1.0):
            theta = -math.pi / 2.0
            psi = math.atan2(-R[0, 1], -R[0, 2])
        else:
            theta = -math.asin(R[2, 0])
            cos_theta = math.cos(theta)
            psi = math.atan2(R[2, 1] / cos_theta, R[2, 2] / cos_theta)
            phi = math.atan2(R[1, 0] / cos_theta, R[0, 0] / cos_theta)
        return psi, theta, phi

    def get_orientation(self, r_vec):
        r_matrix, _ = cv2.Rodrigues(r_vec)
        psi, theta, phi = self.calculate_euler_angles(
            r_matrix)  # roll, pitch, yaw

        def validate_angle(degrees):
            if degrees < 0:
                degrees += 360
            elif degrees > 360:
                degrees -= 360
            return degrees

        psi = validate_angle(math.degrees(psi))
        theta = validate_angle(math.degrees(theta))
        phi = validate_angle(math.degrees(phi))

        return psi, theta, phi

    def get_pose(self, t_vec, r_vec):
        pose = t_vec
        orientation = self.get_orientation(r_vec)
        return pose, orientation

    def get_marker_poses(self, image, camera_matrix, camera_distortion):
        marker_poses = {}

        (corners, ids, rejected_points) = cv2.aruco.detectMarkers(
            image, self.dictionary, parameters=self.aruco_params)

        if len(corners) > 0:
            r_vecs, t_vecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, camera_matrix, camera_distortion)

            image_copy = image.copy()
            cv2.aruco.drawDetectedMarkers(
                image_copy, corners, ids, borderColor=(0, 0, 255))
            cv2.imshow("Frame", image_copy)
            cv2.waitKey(5)


            #TODO: henry make this less bulky thank you :) 
            for i in range(0, len(r_vecs)):
                id = ids[i]
                r_vec = r_vecs[i]
                t_vec = t_vecs[i]
                marker_poses[id] = self.get_pose(t_vec, r_vec)

        return marker_poses, id 

