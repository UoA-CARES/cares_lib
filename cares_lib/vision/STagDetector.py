import stag
from cares_lib.vision.MarkerDetector import MarkerDetector

class STagDetector(MarkerDetector):
    """Detector for STag markers"""
    def __init__(self, marker_size, library_hd=21, error_correction=-1):
        """Initializes STagDetector
        :param int library_hd: The library HD that is used. Possible values are [11, 13, 15, 17, 19, 21, 23].
        :param int error_correction: The amount of error correction that is going to be used.
            Value needs to be in range 0 \<= error_correction \<= (HD-1)/2.
            If set to -1, the max possible value for the given library HD is used.
        """
        self.dictionary = library_hd
        self.error_correction = error_correction
        self.marker_size = marker_size

    def get_marker_poses(self, image, camera_matrix, camera_distortion, display=True):
        (corners, ids, rejected_points) = stag.detectMarkers(image, self.dictionary, self.error_correction)

        return super().get_marker_poses(image, camera_matrix, camera_distortion, corners, ids, display)
