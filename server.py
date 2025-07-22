import numpy as np
import cv2 as cv
import datetime as dt
import os
import socket
import requests

import drone_pb2

from google.protobuf.timestamp_pb2 import Timestamp
from struct import unpack
from ultralytics import YOLO

WIDTH = 640
HEIGHT = 480
CHUNK_SIZE = 4096

BACKEND_URL = "10.42.7.5:4181/api/panels" # TODO: change to exposed endpoint on jetnet

WEIGHTS_PATH = "./weights/yolo_nano_solar_panel.pt"
DEST_DIR = "/home/aicps/tcp_server/images"


class ImageServerProtocol:
    def __init__(self):
        self.socket = None
        self.output_dir = DEST_DIR
        self.model = YOLO(WEIGHTS_PATH)

    def listen(self, server_address):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind(server_address)
        self.socket.listen(1)

    def close(self):
        self.socket.close()
        self.socket = None

    def process_image(self, image_bytes: bytes, metadata: bytes):
        image_array = np.frombuffer(image_bytes, dtype=np.uint8)
        image_array = image_array.reshape((HEIGHT, WIDTH, 3))
        print(image_array.shape)

        timestamp = dt.datetime.now().strftime("%Y%m%d%H%M%S_%f")
        image_path = os.path.join(DEST_DIR, f"{timestamp}.jpg")
        cv.imwrite(image_path, image_array)

        results = self.model.predict(image_array, imgsz=(HEIGHT, WIDTH))
        boxes = results[0].boxes
        update = drone_pb2.PictureUpdate()

        # add metadata to message
        # subject to change as metadata is accepted from drone
        (drone_id, lat, lon, alt, fov_h, fov_v, epoch_sec,) = unpack(">IdddddQ", metadata)
        update.drone_id = drone_id
        pos = drone_pb2.LatLng()
        pos.Latitude = lat
        pos.Longitude = lon
        update.position = pos
        update.altitude = alt
        update.fov_h = fov_h
        update.fov_v = fov_v
        update.message_time = dt.datetime.fromtimestamp(epoch_sec)
        # update.drone_id = 1
        # pos = drone_pb2.LatLng()
        # pos.Latitude = 0
        # pos.Longitude = 0
        # update.position = pos
        # update.altitude = 4
        # update.fov_h = 65
        # update.fov_v = 52
        # update.message_time = Timestamp().GetCurrentTime()

        # add bounding boxes to message
        for xywh, label in zip(boxes.xywh, boxes.cls):
            match label:
                case "healthy":
                    panel_status = "STATUS_CLEAN"
                case "bird_drop" | "bird_feather" | "dust_partical" | "leaf" | "snow":
                    panel_status = "STATUS_DIRTY"                    
                case "cracked":
                    panel_status = "STATUS_CRACKED"
                case _:
                    panel_status = "STATUS_UNSPECIFIED"

            new_bb = drone_pb2.boxes.add()
            new_bb.status = drone_pb2.PanelStatus.Value(panel_status)
            # bounding box in x, y, width, height order
            new_bb.x = xywh[0]
            new_bb.y = xywh[1]

        # add image data + size to message
        update.height = HEIGHT
        update.width = WIDTH
        update.image = image_bytes

        serialized_update = update.SerializeToString()
        
        requests.post(BACKEND_URL, data=serialized_update)

        timestamp = dt.datetime.now().strftime("%Y%m%d%H%M%S_%f")
        image_path = os.path.join(DEST_DIR, f"{timestamp}_pred.jpg")
        results[0].show()
        results[0].save(filename=image_path)

    def receive_images(self):
        try:
            while True:
                connection, client_addr = self.socket.accept()

                try:
                    bs = connection.recv(8)
                    (length,) = unpack(">Q", bs)
                 
                    metadata = connection.recv(52)
                    
                    data = b""
                    while len(data) < length:
                        left = length - len(data)
                        data += connection.recv(
                            CHUNK_SIZE if left > CHUNK_SIZE else left
                        )

                    connection.sendall(b"\00")

                    self.process_image(data, metadata)

                finally:
                    connection.close()
        finally:
            self.close()


if __name__ == "__main__":
    img_server = ImageServerProtocol()
    server_address = ("10.42.7.1", 4567)
    img_server.listen(server_address)
    img_server.receive_images()
