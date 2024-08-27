from deep_sort_realtime.deepsort_tracker import DeepSort
from VehicleDetection import VehicleDetector
import os
import cv2
import numpy as np
import socket
import threading
import time
from queue import Queue

import pathlib
# Ensure proper path handling for Windows
temp = pathlib.PosixPath
pathlib.PosixPath = pathlib.WindowsPath

# Handle library conflicts
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

END_MARKER = b'\xff\xd9'
BUFFER_SIZE = 65355
IMAGE_RESIZE_DIMS = (1280, 720)

# Create a thread-safe queue for storing images
queue_image = Queue()
MAX_QUEUE_SIZE = 3

# Define Direction constants
STRAIGHT = 1
SLOW = 2
STOP = 10

ip_lap = "192.168.1.220"
port_lap = 7000

ip_lap_lead = "192.168.1.36"
port_lap_lead = 7001

model_path = r"C:\Users\Public\ACE\best.pt"
car_detector = VehicleDetector(model_path=model_path,
                               force_reload=False)
object_tracker = DeepSort(max_age=3,
                          n_init=2,
                          nms_max_overlap=1.0,
                          max_cosine_distance=0.3)
focal_lenght = 1100


def create_udp_socket_listen(ip, port):
    """Create and bind a UDP socket listening"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    return sock


def create_udp_socket_send():
    """Create and bind a UDP socket for sending"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, BUFFER_SIZE)
    return sock


def test_image():
    """Receive images via UDP and put them in the queue."""
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    while cap.isOpened():
        _, image = cap.read()
        queue_image.put(image)

        if queue_image.qsize() >= MAX_QUEUE_SIZE:
            queue_image.get()


def get_image():
    """Receive images via UDP and put them in the queue."""
    sock = create_udp_socket_listen(ip_lap, port_lap)
    print(f"Listening on IP {ip_lap}, UDP port {port_lap}")

    image_data = bytearray()
    while True:
        data, _ = sock.recvfrom(BUFFER_SIZE)
        image_data.extend(data)

        if data.endswith(END_MARKER):
            try:
                image_array = np.frombuffer(image_data, dtype=np.uint8)
                image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
                image = cv2.resize(image, (1280, 720))
                queue_image.put(image)

                if queue_image.qsize() >= MAX_QUEUE_SIZE:
                    queue_image.get()
            except Exception as e:
                print(f"Failed to convert image: {str(e)}")

            image_data = bytearray()


def detect_car():
    prev_time = time.time()
    frame_count = 0
    fps = 0
    prev_cmd = STOP

    sock = create_udp_socket_send()

    while True:
        try:
            image = queue_image.get()

            frame_count += 1
            tmp_time = time.time() - prev_time
            if tmp_time >= 1:
                fps = frame_count / tmp_time
                prev_time = time.time()
                frame_count = 0

            results = car_detector.score_frame(image)
            detections = car_detector.plot_boxes(
                results, height=image.shape[0], width=image.shape[1], confidence=0.5)
            tracks = object_tracker.update_tracks(detections, frame=image)

            cmd = STRAIGHT
            for track in tracks:
                if not track.is_confirmed():
                    continue
                track_id = track.track_id
                bbox_x1, bbox_y1, bbox_x2, bbox_y2 = map(int, track.to_ltrb())
                class_label = track.det_class
                conf = track.det_conf if track.det_conf is not None else 0.0

                if bbox_y2 >= image.shape[0] - 200:
                    cmd = STOP

                    # Change color
                    cv2.rectangle(image, (bbox_x1, bbox_y1),
                                  (bbox_x2, bbox_y2), (0, 255, 0), 2)
                else:
                    # Normal tracking display
                    cv2.rectangle(image, (bbox_x1, bbox_y1),
                                  (bbox_x2, bbox_y2), (0, 0, 255), 2)

                cv2.putText(image, "ID: " + str(track_id), (bbox_x1, bbox_y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                cv2.putText(image, f'Class: {
                            class_label}', (bbox_x1, bbox_y1 - 45), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                cv2.putText(image, f'Conf: {
                    conf:.2f}', (bbox_x1, bbox_y1 - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        except Exception as e:
            print("Error processing image:", e)
            cmd = STOP

        if cmd != prev_cmd:
            try:
                data_cmd = f"{cmd}"
                data_bytes_cmd = data_cmd.encode()
                sock.sendto(data_bytes_cmd, (ip_lap_lead, port_lap_lead))
                print(f"Sent UDP message: '{cmd}' to {
                      ip_lap_lead}:{port_lap_lead}")
                prev_cmd = cmd
            except Exception as e:
                print(f"Error sending UDP message: {e}")

        cv2.putText(image, f'FPS: {int(fps)}', (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2)
        cv2.imshow('image', image)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    # sock.close()
    cv2.destroyAllWindows()


def main():
    """Main function to start threads for image processing."""
    threads = [
        threading.Thread(target=get_image,
                         name='GetSendImageThread'),
        threading.Thread(target=detect_car, name='ProcessImageThread'),
    ]

    for thread in threads:
        thread.start()

    for thread in threads:
        thread.join()


if __name__ == "__main__":
    main()
