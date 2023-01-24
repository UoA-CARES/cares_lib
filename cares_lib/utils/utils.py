# General purpose useful functions
import math

def rotation_to_euler(self, rotation_matrix):
  # math is based on http://eecs.qmul.ac.uk/~gslabaugh/publications/euler.pdf
    def is_close(self, x, y, rtol=1.e-5, atol=1.e-8):
      return abs(x - y) <= atol + rtol * abs(y)

    yaw = 0.0
    if is_close(rotation_matrix[2, 0], -1.0):
        pitch = math.pi / 2.0
        roll = math.atan2(rotation_matrix[0, 1], rotation_matrix[0, 2])
    elif is_close(rotation_matrix[2, 0], 1.0):
        pitch = -math.pi / 2.0
        roll = math.atan2(-rotation_matrix[0, 1], -rotation_matrix[0, 2])
    else:
        pitch = -math.asin(rotation_matrix[2, 0])
        cos_theta = math.cos(pitch)
        roll = math.atan2(rotation_matrix[2, 1] / cos_theta, rotation_matrix[2, 2] / cos_theta)
        yaw = math.atan2(rotation_matrix[1, 0] / cos_theta, rotation_matrix[0, 0] / cos_theta)
    return roll, pitch, yaw