import math
import numpy as np
 
def quaternion_to_yaw(q):
    """
    Chuyển quaternion sang yaw (radian).

    Args:
        q (tuple): Quaternion (x, y, z, w)

    Returns:
        float: Góc yaw (radian)
    """
    x, y, z, w = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw