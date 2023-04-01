import math
import numpy as np


def ToEulerAngles(x, y, z, w):

    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.degrees(np.arctan2(sinr_cosp, cosr_cosp))

    # pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch = math.degrees(math.asin(sinp))

    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.degrees(np.arctan2(siny_cosp, cosy_cosp))

    return yaw