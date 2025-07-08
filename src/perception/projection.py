"""
Given a point in 3D space, project it onto the image plane.
"""

import numpy as np

def project_point(xyz, camera_matrix, projection_matrix, w, h):
    """
    Project a 3D point onto the image plane.
    """
    xyz_hom = np.array(tuple(xyz) + (1.0,)).reshape(4, 1)
    view = np.array(camera_matrix).reshape(4, 4).T
    proj = np.array(projection_matrix).reshape(4, 4).T

    clip = proj @ view @ xyz_hom
    clip = clip.flatten()
    clip /= clip[3]

    x = int((clip[0] * 0.5 + 0.5) * w)
    y = int((1 - (clip[1] * 0.5 + 0.5)) * h)
    return x, y