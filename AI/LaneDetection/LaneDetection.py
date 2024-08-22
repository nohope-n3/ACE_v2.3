import numpy as np
import cv2

from CameraCalibration import CameraCalibration
from LaneLines import LaneLines
from PerspectiveTransformation import PerspectiveTransformation
from Thresholding import Thresholding


class LaneDetection:
    def __init__(self):
        self.calibration = CameraCalibration(
            r'AI\LaneDetection\CameraCalibration', 9, 6)
        self.thresholding = Thresholding()
        self.transform = PerspectiveTransformation()
        self.lanelines = LaneLines()

    def forward(self, img):
        out_img = np.copy(img)
        # Undistort the image using camera calibration
        img = self.calibration.undistort(img)
        # Apply perspective transformation to convert image to bird-eye view
        img = self.transform.forward(img)
        # Apply thresholding to extract the lane line
        img = self.thresholding.forward(img)
        # Detect lane lines and motion control
        img = self.lanelines.forward(img)
        # Reverse perspective transformation to normal image
        img = self.transform.backward(img)

        # Combine the original and processed images
        out_img = cv2.addWeighted(out_img, 1, img, 0.6, 0)
        # Visulize detected lane lines
        out_img = self.lanelines.plot(out_img)
        return out_img

    # Process video
    def process_image_rt(self, img):
        out_img = self.forward(img)
        return out_img
