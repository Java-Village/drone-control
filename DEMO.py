#!/usr/bin/env python3

import cv2
import os
import sys
import socket
import argparse
import subprocess
import time
import math

from struct import pack

DRONE_ID = 1
FOV_H = 62.0
FOV_V = 56.0

RECV_MAX_SIZE = 32

def get_position():
    # default for demo purposes
    return (0.0, 0.0)

def get_altitude():
    # TODO: read time-of-flight altimeter
    return 4.0

def capture_image(output_path: str):
    # Remove previous file if it exists
    if os.path.exists(output_path):
        os.remove(output_path)

    # Use libcamera-still to capture image
    cmd = [
        "libcamera-still",
        "-o", output_path,
        "--width", "640",
        "--height", "480",
        "--timeout", "1000"  # 1-second warm-up
    ]
    result = subprocess.run(cmd)
    if result.returncode != 0:
        print("[ERR] libcamera-still failed", file=sys.stderr)
        sys.exit(1)

    # Get metadata right after picture is taken
    location = get_position()
    altitude = get_altitude()
    epoch_seconds = math.floor(time.time())
    metadata = pack('>IdddddQ', DRONE_ID, location[0], location[1], altitude, FOV_H, FOV_V, epoch_seconds)

    # Load image using OpenCV
    img = cv2.imread(output_path)
    if img is None:
        print("[ERR] Failed to load image with OpenCV", file=sys.stderr)
        sys.exit(1)

    print(f"[INFO] Image captured and loaded: {output_path}")
    print(img.shape)
    return img, metadata

def load_test_image(image_path: str):
    # Get metadata right after picture is taken
    location = get_position()
    altitude = get_altitude()
    epoch_seconds = math.floor(time.time())
    metadata = pack('>IdddddQ', DRONE_ID, location[0], location[1], altitude, FOV_H, FOV_V, epoch_seconds)

    # Load image using OpenCV
    img = cv2.imread(image_path)
    if img is None:
        print("[ERR] Failed to load image with OpenCV", file=sys.stderr)
        sys.exit(1)

    print(f"[INFO] Image captured and loaded: {image_path}")
    print(img.shape)
    return img, metadata

def send_image_over_tcp(image: 'np.ndarray', metadata: bytes, server_address: tuple[str, int], sock: socket.socket):
    data = image.tobytes()
    length = pack('>Q', len(data))

    print(f"[INFO] Connecting to {server_address}")
    try:
        sock.connect(server_address)
        # send image data
        sock.sendall(metadata)
        sock.sendall(length)
        sock.sendall(data)
    except Exception as e:
        print(f"[ERR] {e}")
    finally:
        sock.close()
        print("[INFO] Socket closed")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="RPi4B: Capture image and send to Jetson via TCP")
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument(
        "-o", "--output",
        default="images/camera.jpg",
        help="Temporary image name (default: camera.jpg)"
    )
    input_group.add_argument(
        "-i", "--input",
        default="images/test/clean_panels.jpg",
        help="Input filename (for testing) (default: clean_panels.jpg)"
    )
    
    parser.add_argument(
        "--host", default="10.42.7.1", help="Jetson server IP address"
    )
    parser.add_argument(
        "--port", type=int, default=4567, help="Jetson server port"
    )
    args = parser.parse_args()

    outgoing_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = (args.host, args.port)

    incoming_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    incoming_address = ('localhost', 3333)
    incoming_socket.setblocking(False)
    incoming_socket.bind(incoming_address)

    try:
        while True:
            try:
                data, address = incoming_socket.recvfrom(RECV_MAX_SIZE).decode('utf-8')
                if data == "STOP ":
                    # acknowledge packet
                    incoming_socket.sendto("ACK".encode('utf-8'), address)

                    # capture image
                    if args.input:
                        image, metadata = capture_image(args.output)
                    elif args.output:
                        image, metadata = load_test_image(args.input)

                    # send image
                    send_image_over_tcp(image, metadata, server_address, outgoing_socket)

            except socket.error as e:
                if e.errno == socket.errno.EAGAIN or e.errno == socket.errno.EWOULDBLOCK:
                    pass
                else:
                    print(f"Socket error: {e}")

    finally:
        outgoing_socket.close()
        incoming_socket.close()

