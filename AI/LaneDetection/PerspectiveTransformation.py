import cv2
import numpy as np


class PerspectiveTransformation:
    def __init__(self):
        # Define source points for perspective transformation
        self.src = np.float32([(280, 400),    # top-left
                               (30, 650),       # bottom-le
                               (1250, 650),     # bottom-rig
                               (1000, 400)])     # top-rig

        # Define deination points for perspective transformation
        self.dst = np.float32([(180, 0),
                               (180, 720),
                               (1100, 720),
                               (1100, 0)])

        self.M = cv2.getPerspectiveTransform(self.src, self.dst)
        self.M_inv = cv2.getPerspectiveTransform(self.dst, self.src)

    def forward(self, img, img_size=(1280, 720), flags=cv2.INTER_LINEAR):
        return cv2.warpPerspective(img, self.M, img_size, flags=flags)

    def backward(self, img, img_size=(1280, 720), flags=cv2.INTER_LINEAR):
        return cv2.warpPerspective(img, self.M_inv, img_size, flags=flags)
