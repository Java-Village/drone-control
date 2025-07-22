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

def get_position():
    return (0.0, 0.0)

def get_altitude():
    return 4.0

def capture_image(output_path: str) -> 'np.ndarray':
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

def send_image_over_tcp(image: 'np.ndarray', metadata: bytes, server_ip: str, server_port: int):
    # Create TCP socket and send data
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = (server_ip, server_port)

    data = image.tobytes()
    length = pack('>Q', len(data))

    print(f"[INFO] Connecting to {server_ip}:{server_port}")
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
    parser.add_argument(
        "-o", "--output",
        default="image.jpg",
        help="Output filename (default: image.jpg)"
    )
    parser.add_argument(
        "--host", default="10.42.7.1", help="Jetson server IP address"
    )
    parser.add_argument(
        "--port", type=int, default=4567, help="Jetson server port"
    )
    args = parser.parse_args()

    image, metadata = capture_image(args.output)
    send_image_over_tcp(image, metadata, args.host, args.port)

