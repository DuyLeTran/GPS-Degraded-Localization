def project_3d_to_2d(x, y, z, fx, fy, cx, cy):
    """
    Project 3D point (x, y, z) to 2D image plane (u, v) using Pinhole Camera Model.
    """
    if z == 0:
        return 0, 0
    u = (fx * x / z) + cx
    v = (fy * y / z) + cy
    return u, v
