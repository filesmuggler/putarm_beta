#!/usr/bin/env python

import rospy as rp
import numpy as np
from sensor_msgs.msg import Image

class Dataset:
    def __init__(self):
        self.img_1 = None
        self.img_2 = None
        rp.init_node('dataset_creator', anonymous=True)
        rp.Subscriber("/logitech/image_raw/", Image, self.img_1_callback)
        rp.Subscriber("/realsense/image_raw/", Image, self.img_2_callback)

    def img_1_callback(self, data):
        im = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        self.img_1 = im
    def img_2_callback(self, data):
        im = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        self.img_2 = im

    def save(self):
        print("IMG 1:", self.img_1.shape)
        print("IMG 2:", self.img_2.shape)


if __name__ == '__main__':
    ds = Dataset()
    rp.sleep(1)
    for i in range(10):
        ds.save()
        rp.sleep(1)


