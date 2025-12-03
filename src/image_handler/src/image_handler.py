#!/usr/bin/env python3
import socket
import time
import math
import rospy
import requests

from sensor_msgs.msg import Image

from struct import pack

DRONE_ID = 1
FOV_H = 62.0
FOV_V = 56.0

RECV_MAX_SIZE = 32

RATE = 30 # Hz
CLOVER_RAW_IMG_TOPIC = "/main_camera/image_raw"
JETSON_PORT = 4567
JETSON_IP = "10.42.0.1"
BACKEND_API = "10.42.0.111:4181/api/drones"

class DroneImageNode:
    def __init__(self, address, port):
        # initialize node
        self.node = rospy.init_node('image_handler')
        # no subscriber needed -- want single messages
        self.rate = rospy.Rate(RATE)
        self.address = address
        self.port = port

    def begin(self):
        # initialize socket connections
        # intended for use with `with ... as`
        self.server_address = (self.address, self.port)

        self.outgoing_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rospy.loginfo(f"[INFO] Connecting to {self.server_address}")
        self.outgoing_socket.connect(self.server_address)

        self.incoming_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.incoming_address = ('', 3333)
        self.incoming_socket.setblocking(False)
        self.incoming_socket.bind(self.incoming_address)

        self.current_task = None

    def end(self):
        self.outgoing_socket.close()
        self.incoming_socket.close()
        rospy.loginfo("sockets closed")

    def get_position(self):
        # default for demo purposes
        return (0.0, 0.0)

    def get_altitude(self):
        # TODO: read time-of-flight altimeter
        return 1.8 # meters ... ?

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

        rospy.loginfo(f"[INFO] Image captured: ({image.height}, {image.width})")

        return image.data, metadata

    def send_image_over_tcp(self, image, metadata, sock):
        rospy.loginfo(type(image))
        data = image
        length = pack('>Q', len(data))

        try:
            # send image data
            sock.sendall(metadata)
            sock.sendall(length)
            sock.sendall(data)
        except Exception as e:
            rospy.logerr(f"[ERR] {e}")

    def run(self):
        while not rospy.is_shutdown():
            # try:
                # data, address = self.incoming_socket.recvfrom(RECV_MAX_SIZE)
            # if data.decode('UTF-8') == "STOP ":
            input()
            # acknowledge packet
            rospy.loginfo("got packet from ksw")
            # incoming_socket.sendto("ACK".encode('utf-8'), address)

            # capture image
            rospy.loginfo("capturing image")
            image, metadata = self.capture_image()

            # send image
            rospy.loginfo("sending image over TCP")
            self.send_image_over_tcp(image, metadata, self.outgoing_socket)

            self.rate.sleep()

            # except socket.error as e:
                # if e.errno == socket.errno.EAGAIN or e.errno == socket.errno.EWOULDBLOCK:
                    # self.rate.sleep()
                    # pass
                # else:
                    # rospy.logerror(f"Socket error: {e}")

if __name__ == "__main__":
    image_handler_node = DroneImageNode(JETSON_IP, JETSON_PORT)
    image_handler_node.begin()
    try:
        image_handler_node.run()
    finally:
        image_handler_node.end()
