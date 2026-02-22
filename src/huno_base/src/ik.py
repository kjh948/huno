import numpy as np

def leg_ik(x, z, l1=0.10, l2=0.10):
    """
    2-link planar IK
    """
    D = (x**2 + z**2 - l1**2 - l2**2) / (2*l1*l2)
    D = np.clip(D, -1.0, 1.0)

    knee = np.arccos(D)
    hip = np.arctan2(z, x) - np.arctan2(l2*np.sin(knee), l1 + l2*np.cos(knee))
    ankle = -hip - knee

    return hip, knee, ankle
