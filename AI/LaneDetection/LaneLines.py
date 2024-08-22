import cv2
import numpy as np

# Define Direction constants
STRAIGHT = 1
SLOW = 2
LEFT = 3
RIGHT = 4
TURNLEFT = 5
TURNRIGHT = 6
FORWARD = 7
BACKWARD = 8
STANDBY = 9
STOP = 10

# Define Direction constants
commands = [
    "Straght",      # 1
    "Slow",         # 2
    "Left",         # 3
    "Right",        # 4
    "TurnLeft",     # 5
    "TunrRight",    # 6
    "Forward",      # 7
    "backward",     # 8
    "standby",      # 9
    "stop"          # 10
]


def hist(img):
    bottom_half = img[img.shape[0]//2:, :]
    return np.sum(bottom_half, axis=0)


class LaneLines:
    def __init__(self):
        self.left_fit = None
        self.right_fit = None
        self.binary = None
        self.nonzero = None
        self.nonzerox = None
        self.nonzeroy = None
        self.clear_visibility = True
        self.dir = []
        self.msg = STRAIGHT
        self.pos = 0

        self.nwindows = 9
        self.margin = 30
        self.minpix = 50

    def forward(self, img):
        self.extract_features(img)
        return self.fit_poly(img)

    def pixels_in_window(self, center, margin, height):
        topleft = (center[0]-margin, center[1]-height//2)
        bottomright = (center[0]+margin, center[1]+height//2)

        condx = (topleft[0] <= self.nonzerox) & (
            self.nonzerox <= bottomright[0])
        condy = (topleft[1] <= self.nonzeroy) & (
            self.nonzeroy <= bottomright[1])
        return self.nonzerox[condx & condy], self.nonzeroy[condx & condy]

    def extract_features(self, img):
        self.img = img
        self.window_height = int(img.shape[0]//self.nwindows)

        self.nonzero = img.nonzero()
        self.nonzerox = np.array(self.nonzero[1])
        self.nonzeroy = np.array(self.nonzero[0])

    def find_lane_pixels(self, img):
        assert (len(img.shape) == 2)
        out_img = np.dstack((img, img, img))

        histogram = hist(img)
        midpoint = histogram.shape[0]//2
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        leftx_current = leftx_base
        rightx_current = rightx_base
        y_current = img.shape[0] + self.window_height//2

        leftx, lefty, rightx, righty = [], [], [], []

        for _ in range(self.nwindows):
            y_current -= self.window_height
            center_left = (leftx_current, y_current)
            center_right = (rightx_current, y_current)

            good_left_x, good_left_y = self.pixels_in_window(
                center_left, self.margin, self.window_height)
            good_right_x, good_right_y = self.pixels_in_window(
                center_right, self.margin, self.window_height)

            leftx.extend(good_left_x)
            lefty.extend(good_left_y)
            rightx.extend(good_right_x)
            righty.extend(good_right_y)

            if len(good_left_x) > self.minpix:
                leftx_current = np.int32(np.mean(good_left_x))
            if len(good_right_x) > self.minpix:
                rightx_current = np.int32(np.mean(good_right_x))

        return leftx, lefty, rightx, righty, out_img

    def fit_poly(self, img):
        leftx, lefty, rightx, righty, out_img = self.find_lane_pixels(img)

        if len(lefty) > 1500:
            self.left_fit = np.polyfit(lefty, leftx, 2)
        if len(righty) > 1500:
            self.right_fit = np.polyfit(righty, rightx, 2)

        maxy = img.shape[0] - 1
        miny = img.shape[0] // 3
        if len(lefty):
            maxy = max(maxy, np.max(lefty))
            miny = min(miny, np.min(lefty))

        if len(righty):
            maxy = max(maxy, np.max(righty))
            miny = min(miny, np.min(righty))

        ploty = np.linspace(miny, maxy, img.shape[0])

        left_fitx = self.left_fit[0]*ploty**2 + \
            self.left_fit[1]*ploty + self.left_fit[2]
        right_fitx = self.right_fit[0]*ploty**2 + \
            self.right_fit[1]*ploty + self.right_fit[2]

        for i, y in enumerate(ploty):
            l = int(left_fitx[i])
            r = int(right_fitx[i])
            y = int(y)
            cv2.line(out_img, (l, y), (r, y), (0, 255, 0))

        return out_img

    def plot(self, out_img):
        np.set_printoptions(precision=6, suppress=True)
        lR, rR, self.pos = self.measure_curvature()
        #  curvature_msg = "Curvature = {:.0f} m".format(min(lR, rR))

        # value = None
        # if abs(self.left_fit[0]) > abs(self.right_fit[0]):
        #     value = self.left_fit[0]
        # else:
        #     value = self.right_fit[0]
        # if abs(value) <= 0.0005:
        #     self.dir.append('F')
        # elif value < 0:
        #     self.dir.append('L')
        # else:
        #     self.dir.append('R')

        # if len(self.dir) > 10:
        #     self.dir.pop(0)

        # direction = max(set(self.dir), key=self.dir.count)

        # if direction == 'L':
        #     self.msg = TURNLEFT
        # if direction == 'R':
        #     self.msg = TURNRIGHT
        # if direction == 'F':
        #     if (self.pos > 0.1):
        #         self.msg = LEFT
        #     elif (self.pos < -0.1):
        #         self.msg = RIGHT
        #     else:
        #         self.msg = STRAIGHT

        if (self.pos > 0.1):
            self.msg = LEFT
        elif (self.pos < -0.1):
            self.msg = RIGHT
        else:
            self.msg = STRAIGHT

        cv2.putText(out_img, commands[self.msg-1], org=(10, 240), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=1, color=(255, 255, 255), thickness=2)

        # if direction in 'LR':
        #     cv2.putText(out_img, curvature_msg, org=(
        #         10, 280), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255, 255, 255), thickness=2)

        cv2.putText(
            out_img,
            "Vehicle is {:.2f} m away from center".format(self.pos),
            org=(10, 310),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.66,
            color=(255, 255, 255),
            thickness=2)

        return out_img

    def measure_curvature(self):
        ym = 30/720
        xm = 3.7/700

        left_fit = self.left_fit.copy()
        right_fit = self.right_fit.copy()
        y_eval = 700 * ym

        left_curveR = (
            (1 + (2*left_fit[0] * y_eval + left_fit[1])**2)**1.5) / np.absolute(2*left_fit[0])
        right_curveR = (
            (1 + (2*right_fit[0]*y_eval + right_fit[1])**2)**1.5) / np.absolute(2*right_fit[0])

        xl = np.dot(self.left_fit, [700**2, 700, 1])
        xr = np.dot(self.right_fit, [700**2, 700, 1])
        pos = (1280//2 - (xl+xr)//2)*xm

        return left_curveR, right_curveR, pos

    def handle_lane_detect_command(self, speed_cmd, x_battery, x_motor):
        speed = 0
        alpha = 0

        if self.msg == STRAIGHT:
            alpha = x_motor
            speed = speed_cmd

        if self.msg == LEFT:
            if self.pos >= 0.5:
                alpha = 35 + x_battery
                speed = speed_cmd - 45 + x_battery
            elif self.pos >= 0.4:
                alpha = 30 + x_battery
                speed = speed_cmd - 40 + x_battery
            elif self.pos >= 0.3:
                alpha = 25 + x_battery
                speed = speed_cmd - 30 + x_battery
            elif self.pos >= 0.2:
                alpha = 20 + x_battery
                speed = speed_cmd - 20 + x_battery
            elif self.pos >= 0.1:
                alpha = 15 + x_battery
                speed = speed_cmd - 15 + x_battery

        if self.msg == RIGHT:
            if self.pos <= -0.5:
                alpha = 35 + x_battery
                speed = speed_cmd - 45 + x_battery
            elif self.pos <= -0.4:
                alpha = 30 + x_battery
                speed = speed_cmd - 40 + x_battery
            elif self.pos <= -0.3:
                alpha = 25 + x_battery
                speed = speed_cmd - 30 + x_battery
            elif self.pos <= -0.2:
                alpha = 20 + x_battery
                speed = speed_cmd - 20 + x_battery
            elif self.pos <= -0.1:
                alpha = 15 + x_battery
                speed = speed_cmd - 15 + x_battery
            speed += x_motor
            alpha += x_motor

        command = f"{self.msg} {speed} {int(alpha)}"

        return command
