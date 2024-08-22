import torch
from PIL import Image
import cv2


class SignDetector():
    def __init__(self, model_path, force_reload):
        self.model = self.load_model(model_path, force_reload)
        self.classes = self.model.names
        print(self.classes)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)
        print("Using Device: ", self.device)

    def load_model(self, model_path, force_reload):
        if model_path:
            model = torch.hub.load(
                'ultralytics/yolov5',
                'custom',
                path=model_path,
                force_reload=force_reload
            )
        else:
            model = torch.hub.load(
                'ultralytics/yolov5',
                'yolov5s',
                pretrained=True
            )

        return model

    def score_frame(self, frame):
        # downscale_factor = 2
        # width = int(frame.shape[1] / downscale_factor)
        # height = int(frame.shape[0] / downscale_factor)
        # frame = cv2.resize(frame, (width, height))
        # frame = frame.to(self.device)

        frame_rgb = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        results = self.model(frame_rgb)
        # print('Results:', results)

        labels, cord = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]
        # print('Lable:', labels)
        # print('Cord:', cord)

        return labels, cord

    def class_to_label(self, x):
        return self.classes[int(x)]

    def plot_boxes(self, results, height, width, confidence=0.3):
        labels, cord = results
        detections = []

        for i in range(len(labels)):
            row = cord[i]
            if row[4] >= confidence:
                x1, y1, x2, y2 = int(
                    row[0]*width), int(row[1]*height), int(row[2]*width), int(row[3]*height)

                conf = float(row[4].item())
                class_label = self.class_to_label(labels[i])
                # print(feature)
                detections.append(
                    ([x1, y1, int(x2-x1), int(y2-y1)], conf, class_label))

        return detections

    def calculate_distance(self, object_size_pixels):
        object_size_mm = 30
        sensor_width_mm = 4.8
        image_width_pixels = 640
        focal_length_mm = 2.4

        # Calculate pixel size in mm
        pixel_size_mm = sensor_width_mm / image_width_pixels

        # Calculate distance using the formula
        distance_mm = (object_size_mm * focal_length_mm) / \
            (object_size_pixels * pixel_size_mm)

        return int(distance_mm)
