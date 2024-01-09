import cv2
from cares_lib.vision.MarkerDetector import MarkerDetector

class ArucoDetector(MarkerDetector):
    def __init__(self, marker_size, dictionary_id=cv2.aruco.DICT_4X4_50):
        super().__init__(marker_size)
        self.dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    def get_marker_poses(self, image, camera_matrix, camera_distortion, display=True):
        (corners, ids, rejected_points) = cv2.aruco.detectMarkers(
            image, self.dictionary, parameters=self.aruco_params)

        return super().get_marker_poses(image, camera_matrix, camera_distortion, corners, ids, display)
