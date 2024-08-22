import numpy as np
import cv2
import glob
import matplotlib.image as mpimage


class CameraCalibration():
    def __init__(self, image_dir, nx, ny, debug=False):
        fnames = glob.glob("{}/*".format(image_dir))
        objpoints = []
        imagepoints = []

        # Coordinates of chessboard's corners in 3D
        objp = np.zeros((nx*ny, 3), np.float32)
        objp[:, :2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)

        # Go through all chessboard images
        for f in fnames:
            image = mpimage.imread(f)

            # Find chessboard corners
            ret, corners = cv2.findChessboardCorners(image, (nx, ny))
            if ret:
                imagepoints.append(corners)
                objpoints.append(objp)

        shape = (image.shape[1], image.shape[0])
        ret, self.mtx, self.dist, _, _ = cv2.calibrateCamera(
            objpoints, imagepoints, shape, None, None)

        if not ret:
            raise Exception("Unable to calibrate camera")

    def undistort(self, image):
        # Convert to grayscale image
        return cv2.undistort(image, self.mtx, self.dist, None, self.mtx)
