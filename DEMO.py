#!/usr/bin/env python3

# TODO: turn this into a ROS node
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

def capture_image():
    cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L)
    retval, img = cap.read()
    if not retval:
        print(f"[ERR] could not read from camera (return value is false)")

    
    # Get metadata right after picture is taken
    location = get_position()
    altitude = get_altitude()
    epoch_seconds = math.floor(time.time())
    metadata = pack('>IdddddQ', DRONE_ID, location[0], location[1], altitude, FOV_H, FOV_V, epoch_seconds)

    print(f"[INFO] Image captured: {img.shape}")

    return img, metadata

def load_test_image(image_path):
    # Load image using OpenCV
    img = cv2.imread(image_path)
    if img is None:
        print("[ERR] Failed to load image with OpenCV", file=sys.stderr)
        sys.exit(1)

    # Get metadata right after picture is taken
    location = get_position()
    altitude = get_altitude()
    epoch_seconds = math.floor(time.time())
    metadata = pack('>IdddddQ', DRONE_ID, location[0], location[1], altitude, FOV_H, FOV_V, epoch_seconds)


    print(f"[INFO] Image loaded: {image_path} is {img.shape}")
    return img, metadata

def send_image_over_tcp(image, metadata, sock):
    data = image.tobytes()
    length = pack('>Q', len(data))

    try:
        # send image data
        sock.sendall(metadata)
        sock.sendall(length)
        sock.sendall(data)
    except Exception as e:
        print(f"[ERR] {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="RPi4B: Capture image and send to Jetson via TCP")
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument(
        "-t", "--test-image",
        default="images/test/clean_panels.jpg",
        help="Test mode. Uses the image provided as an argument (default: images/test/clean_panels.jpg)"
    )
    input_group.add_argument(
        "-c", "--camera",
        action='store_true',
        help="Camera mode. Takes a picture with the attached camera."
    )
    
    parser.add_argument(
        "--host", default="10.42.7.1", help="Jetson server IP address"
    )
    parser.add_argument(
        "--port", type=int, default=4567, help="Jetson server port"
    )
    args = parser.parse_args()
    print(args)

    outgoing_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = (args.host, args.port)
    print(f"[INFO] Connecting to {server_address}")
    outgoing_socket.connect(server_address)

    incoming_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    incoming_address = ('', 3333)
    incoming_socket.setblocking(False)
    incoming_socket.bind(incoming_address)

    try:
        while True:
            try:
                data, address = incoming_socket.recvfrom(RECV_MAX_SIZE)
                if data.decode('UTF-8') == "STOP ":
                    # acknowledge packet
                    print("got packet from ksw")
                    incoming_socket.sendto("ACK".encode('utf-8'), address)

                    # capture image
                    print("capturing image")
                    if args.camera:
                        image, metadata = capture_image()
                    elif args.test_image:
                        image, metadata = load_test_image(args.test_image)

                    # send image
                    print("sending image over TCP")
                    send_image_over_tcp(image, metadata, outgoing_socket)

            except socket.error as e:
                if e.errno == socket.errno.EAGAIN or e.errno == socket.errno.EWOULDBLOCK:
                    pass
                else:
                    print(f"Socket error: {e}")

    finally:
        outgoing_socket.close()
        incoming_socket.close()
        print("sockets closed")

