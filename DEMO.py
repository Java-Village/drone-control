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
import rospy

from sensor_msgs.msg import Image

from struct import pack

DRONE_ID = 1
FOV_H = 62.0
FOV_V = 56.0

RECV_MAX_SIZE = 32

CLOVER_RAW_IMG_TOPIC = "" # TODO
JETSON_PORT = 4567
JETSON_IP = "10.42.7.1"

class DroneImageNode:
    

    def get_position():
        # default for demo purposes
        return (0.0, 0.0)

    def get_altitude():
        # TODO: read time-of-flight altimeter
        return 4.0

    def capture_image(self):
        image = rospy.wait_for_message(CLOVER_RAW_IMG_TOPIC, Image)
        # cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L)
        # retval, img = cap.read()
        # if not retval:
        #     print("[ERR] could not read from camera (return value is false)")
    
        # Get metadata right after picture is taken
        location = self.get_position()
        altitude = self.get_altitude()
        epoch_seconds = math.floor(time.time())
        metadata = pack('>IdddddQ', DRONE_ID, location[0], location[1], altitude, FOV_H, FOV_V, epoch_seconds)

        print(f"[INFO] Image captured: ({image.height}, {image.width})")

        return image.data, metadata

    def send_image_over_tcp(self, image, metadata, sock):
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
        outgoing_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = (JETSON_IP, JETSON_PORT)
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
                        image, metadata = capture_image()

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

