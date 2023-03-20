import logging
import cv2
import numpy as np

class Camera(object):
    def __init__(self, camera_id, camera_matrix_path, camera_distortion_path):

        self.camera = cv2.VideoCapture(camera_id)
        if not self.camera.isOpened():
            raise IOError("Could not open video device")

        self.camera_matrix     = np.loadtxt(camera_matrix_path)
        self.camera_distortion = np.loadtxt(camera_distortion_path)

    def get_frame(self, delay_frame=5):
        returned = False
        frame = None

        # read delay_frame times needed because frame delay. MUST BE INCLUDED
        for _ in range(0, delay_frame):
            returned, frame = self.camera.read()

        if not returned or frame is None:
            raise IOError("Camera did not return a frame")    

        return frame