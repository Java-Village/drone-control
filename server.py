import numpy as np
import cv2 as cv
import datetime as dt
import os
import socket
import requests
import math

import drone_pb2

from google.protobuf.timestamp_pb2 import Timestamp
from struct import unpack
from ultralytics import YOLO

WIDTH = 640
HEIGHT = 480
CHUNK_SIZE = 4096

BACKEND_URL = "http://10.42.7.111:4181/api/panels"

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
        image_path = os.path.join(DEST_DIR, f"{timestamp}.png")
        cv.imwrite(image_path, image_array)

        print("runnning prediction model")
        results = self.model.predict(image_array)
        boxes = results[0].boxes
        update = drone_pb2.PictureUpdate()
        timestamp = dt.datetime.now().strftime("%Y%m%d%H%M%S_%f")
        image_path = os.path.join(DEST_DIR, f"{timestamp}_pred.png")
        results[0].show()
        results[0].save(filename=image_path)

        # add metadata to message
        # subject to change as metadata is accepted from drone
        (drone_id, lat, lon, alt, fov_h, fov_v, epoch_sec,) = unpack(">IdddddQ", metadata)
        update.drone_id = drone_id
        update.position.latitude = lat
        update.position.longitude = lon
        update.altitude = alt
        update.fov_h = fov_h
        update.fov_v = fov_v
        update.message_time = dt.datetime.fromtimestamp(epoch_sec)

        # add bounding boxes to message
        for xywh, label in zip(boxes.xywh, boxes.cls):
            match self.model.names[int(label)]:
                case "healthy":
                    panel_status = "STATUS_HEALTHY"
                case "bird_drop" | "bird_feather" | "dust_partical" | "leaf" | "snow":
                    panel_status = "STATUS_DIRTY"                    
                case "cracked":
                    panel_status = "STATUS_CRACKED"
                case _:
                    panel_status = "STATUS_UNSPECIFIED"

            np_xywh = xywh.cpu().numpy()
            new_bb = update.boxes.add()
            new_bb.status = drone_pb2.PanelStatus.Value(panel_status)
            # bounding box in x, y, width, height order
            new_bb.x = int(math.floor(np_xywh[0]))
            new_bb.y = int(math.floor(np_xywh[1]))

        # add image data + size to message
        update.height = HEIGHT
        update.width = WIDTH
        with open(image_path, "rb") as png_bytes:
            update.image = png_bytes.read()

        serialized_update = update.SerializeToString()

        
        print("posting update to backend")
        r = requests.post(BACKEND_URL, data=serialized_update)
        print(r.text)


    def receive_images(self):
        try:
            while True:
                print("waiting to recieve image")
                connection, client_addr = self.socket.accept()

                try:
                    metadata = connection.recv(52)

                    bs = connection.recv(8)
                    (length,) = unpack(">Q", bs)
                    print(length)
                 
                    
                    data = b""
                    while len(data) < length:
                        left = length - len(data)
                        data += connection.recv(
                            CHUNK_SIZE if left > CHUNK_SIZE else left
                        )

                    connection.sendall(b"\00")

                    print("recieved image + metadata")
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
