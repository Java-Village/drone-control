import numpy as np
import cv2 as cv
import datetime as dt
import os
import torch
import socket

from struct import unpack
from ultralytics import YOLO

WIDTH = 640
HEIGHT = 480
CHUNK_SIZE = 4096

WEIGHTS_PATH = "./weights/yolo_nano_solar_panel.pt"
DEST_DIR = "/home/aicps//tcp_server/images"

class ImageServerProtocol:
    def __init__(self):
        self.socket = None
        self.output_dir = DEST_DIR

    def listen(self, server_address):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind(server_address)
        self.socket.listen(1)

    def close(self):
        self.socket.close()
        self.socket = None

    def save_image(self, image_bytes: bytes):
        image_array = np.frombuffer(image_bytes, dtype=np.uint8)
        image_array = image_array.reshape((HEIGHT, WIDTH, 3))
        print(image_array.shape)

        timestamp = dt.datetime.now().strftime("%Y%m%d%H%M%S_%f")
        image_path = os.path.join(DEST_DIR, f"{timestamp}.jpg")
        cv.imwrite(image_path, image_array)

        return image_array

    def process_image(self, image_array):
        model = YOLO(WEIGHTS_PATH)
        results = model.predict(image_array, imgsz=(HEIGHT, WIDTH))
        results[0].show()
        timestamp = dt.datetime.now().strftime("%Y%m%d%H%M%S_%f")
        image_path = os.path.join(DEST_DIR, f"{timestamp}_pred.jpg")
        results[0].save(filename=image_path)

    def receive_images(self):

        try:
            while True:
                connection, client_addr = self.socket.accept()

                try:
                    bs = connection.recv(8)
                    (length,) = unpack('>Q', bs)
                    data = b''
                    print(length)

                    while len(data) < length:
                        left = length - len(data)
                        data += connection.recv(CHUNK_SIZE if left > CHUNK_SIZE else left)
                    
                    connection.sendall(b'\00')

                    image_array = self.save_image(data)
                    self.process_image(image_array)

                finally:
                    connection.close()
        finally:
            self.close()

if __name__ == "__main__":
    img_server = ImageServerProtocol()
    server_address = ('10.42.7.1', 4567)
    img_server.listen(server_address)
    img_server.receive_images()
