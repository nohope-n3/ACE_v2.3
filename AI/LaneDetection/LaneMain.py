import cv2
import numpy as np
import time
import threading
import socket
from queue import Queue
from LaneDetection import LaneDetection

# Constants
# Define Image constants
IMAGE_RESIZE_DIMS = (1280, 720)
END_MARKER = b'\xff\xd9'
BUFFER_SIZE = 2048

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

# Define Speed constants
SLOW = 100
NORMAL = 150
FAST = 200
SPEED_CMD = 165
X_BATTERY = 0
X_MOTOR = 0

ip_lap_lead = "192.168.1.36"
port_lap_lead = 3000
port_get_member_car = 7001
port_get_member_sign = 8001

ip_cam = "192.168.1.167"
port_cam = 3001

ip_lap_member_car = "192.168.1.220"
port_lap_member_car = 7000

ip_lap_member_sign = "192.168.1.129"
port_lap_member_sign = 8000


# Create a thread-safe queue for storing images
queue_image = Queue()
lane_detection = LaneDetection()
MAX_QUEUE_SIZE = 3


cmd_member_car = STRAIGHT
cmd_member_sign = STRAIGHT


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


def handle_control_command():
    if STOP in (cmd_member_car, cmd_member_sign):
        cmd = f"{STOP} {SPEED_CMD} {0}"
        return cmd

    if SLOW in (cmd_member_car, cmd_member_sign):
        cmd = f"{SLOW} {SPEED_CMD} {20}"
        return cmd

    if cmd_member_sign == STANDBY:
        cmd = f"{STANDBY} {0} {30}"
        return cmd

    if cmd_member_sign == FORWARD:
        cmd = f"{STANDBY} {SPEED_CMD + 10} {5}"
        return cmd

    if cmd_member_sign in (TURNLEFT, TURNRIGHT):
        cmd = f"{cmd_member_sign} {120 - X_BATTERY} {8}"
        return cmd

    cmd = lane_detection.lanelines.handle_lane_detect_command(
        SPEED_CMD, X_BATTERY, X_MOTOR)
    return cmd


def test_get_image():
    """Receive images via UDP and put them in the queue."""
    sock = create_udp_socket_listen(ip_lap_lead, port_lap_lead)
    print(f"Listening on IP {ip_lap_lead}, UDP port {port_lap_lead}")

    image_data = bytearray()

    while True:
        data, _ = sock.recvfrom(BUFFER_SIZE)
        image_data.extend(data)

        if data.endswith(END_MARKER):
            try:
                image_array = np.frombuffer(image_data, dtype=np.uint8)
                image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
                image = cv2.resize(image, IMAGE_RESIZE_DIMS)
                queue_image.put(image)

                if queue_image.qsize() >= MAX_QUEUE_SIZE:
                    queue_image.get()
            except Exception as e:
                print(f"Failed to convert image: {str(e)}")

            image_data = bytearray()


def test_lane_detection():
    prev_time = time.time()
    frame_count = 0
    fps = 0
    prev_cmd = f"{STOP} {0} {0}"

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

            image = lane_detection.process_image_rt(image)
            cmd = lane_detection.lanelines.handle_lane_detect_command(
                SPEED_CMD, X_BATTERY, X_MOTOR)

        except Exception as e:
            # print("Error processing image:", e)
            cv2.putText(image, "None", (150, 30),
                        cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
            cmd = f"{STOP} {0} {0}"

        # Send command to car kit
        if cmd != prev_cmd:
            try:
                data_bytes_cmd = cmd.encode()
                sock.sendto(data_bytes_cmd, (ip_cam, port_cam))
                print(f"Sent UDP message: '{cmd}' to {ip_cam}:{port_cam}")
                prev_cmd = cmd

            except Exception as e:
                print(f"Error sending UDP message: {e}")

        # Show FPS
        cv2.putText(image, f"FPS: {int(fps)}", (10, 30),
                    cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
        cv2.imshow("Detected Image", image)

        # End thread
        if cv2.waitKey(1) & 0xFF == 27:
            cmd = f"{STOP} {0} {0}"
            data_bytes = cmd.encode()

            try:
                sock.sendto(data_bytes, (ip_cam, port_cam))
                print(f"Sent final UDP message: '{
                      cmd}' to {ip_cam}: {port_cam}")
            except Exception as e:
                print(f"Error sending final UDP message: {e}")
            break

    sock.close()
    cv2.destroyAllWindows()


def get_send_image():
    """Receive images from the leader and send them to members."""
    socket_lead = create_udp_socket_listen(ip_lap_lead, port_lap_lead)
    print(f"Listening on IP {ip_lap_lead}, UDP port {port_lap_lead}")

    socket_member = create_udp_socket_send()
    image_data = bytearray()

    while True:
        data, _ = socket_lead.recvfrom(BUFFER_SIZE)
        image_data.extend(data)

        if data.endswith(END_MARKER):
            try:
                socket_member.sendto(
                    image_data, (ip_lap_member_car, port_lap_member_car))
                socket_member.sendto(
                    image_data, (ip_lap_member_sign, port_lap_member_sign))

                image_array = np.frombuffer(image_data, dtype=np.uint8)
                image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError(
                        "Image decoding failed. The image data may be corrupted.")

                image = cv2.resize(image, IMAGE_RESIZE_DIMS)

                queue_image.put(image)
                if queue_image.qsize() >= MAX_QUEUE_SIZE:
                    queue_image.get()

            except Exception as e:
                print(f"Failed to convert image: {str(e)}")

            finally:
                image_data = bytearray()


def get_cmd_car():
    global cmd_member_car
    member_socket_1 = create_udp_socket_listen(
        ip_lap_lead, port_get_member_car)
    print(f'Listening at: {(ip_lap_lead, port_get_member_car)}')

    while True:
        cmd, _ = member_socket_1.recvfrom(BUFFER_SIZE)
        cmd_member_car = int(cmd)
        print('Car: ', cmd_member_car)


def get_cmd_sign():
    global cmd_member_sign
    member_socket_2 = create_udp_socket_listen(
        ip_lap_lead, port_get_member_sign)
    print(f'Listening at: {(ip_lap_lead, port_get_member_sign)}')

    while True:
        cmd, _ = member_socket_2.recvfrom(BUFFER_SIZE)
        cmd_member_sign = int(cmd)
        print('Sign: ', cmd_member_sign)


def process_image():
    """Process images for lane detection and send commands."""
    prev_time = time.time()
    frame_count = 0
    fps = 0
    prev_cmd = f"{STOP} {0} {0}"

    sock_cam = create_udp_socket_send()
    while True:
        try:
            image = queue_image.get()

            frame_count += 1
            tmp_time = time.time() - prev_time
            if tmp_time >= 1:
                fps = frame_count / tmp_time
                prev_time = time.time()
                frame_count = 0

            # Process command
            image = lane_detection.process_image_rt(image)
            cmd = handle_control_command()
            # cmd = f"{STRAIGHT} {150} {0}"

        except Exception as e:
            # print("Error processing image:", e)
            cv2.putText(image, "None", (150, 30),
                        cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
            cmd = f"{STOP} {0} {0}"

        # Send command to car kit
        if cmd != prev_cmd:
            try:
                data_bytes_cmd = cmd.encode()
                sock_cam.sendto(data_bytes_cmd, (ip_cam, port_cam))
                print(f"Sent UDP message: '{cmd}' to {ip_cam}:{port_cam}")
                prev_cmd = cmd

            except Exception as e:
                print(f"Error sending UDP message: {e}")

        cv2.putText(image, f"FPS: {int(fps)}", (10, 30),
                    cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
        cv2.imshow("Detected Image", image)

        if cv2.waitKey(1) & 0xFF == 27:
            cmd = f"{STOP} {0} {0}"
            data_bytes = cmd.encode()

            try:
                sock_cam.sendto(data_bytes, (ip_cam, port_cam))
                print(f"Sent final UDP message: '{
                      cmd}' to {ip_cam}: {port_cam}")
            except Exception as e:
                print(f"Error sending final UDP message: {e}")
            break

    sock_cam.close()
    cv2.destroyAllWindows()


def main():
    """Main function to start threads for image processing."""
    # threads = [
    #     threading.Thread(target=test_get_image,
    #                      name='GetSendImageThread'),
    #     threading.Thread(target=test_lane_detection,
    #                      name='ProcessImageThread')
    # ]

    threads = [
        threading.Thread(target=get_send_image,
                         name='GetSendImageThread'),
        threading.Thread(target=process_image,
                         name='ProcessImageThread'),
        threading.Thread(target=get_cmd_sign,
                         name='GetcmdTrafficSign'),
        threading.Thread(target=get_cmd_car,
                         name='GetcmdCar')
    ]

    for thread in threads:
        thread.start()

    for thread in threads:
        thread.join()


if __name__ == "__main__":
    main()
