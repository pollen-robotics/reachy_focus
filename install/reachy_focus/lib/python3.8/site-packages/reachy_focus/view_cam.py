"""
Example showing camera image
"""
import logging
import time

import numpy as np
import cv2 as cv

import rclpy
from rclpy.node import Node

from sensor_msgs.msg._compressed_image import CompressedImage


class ImageViewer(Node):
    def __init__(self) -> None:
        super().__init__('image_viewer')

        self.left_image_sub = self.create_subscription(
            msg_type=CompressedImage,
            topic='left_image',
            callback=self.left_image_cb,
            qos_profile=1,
        )

        self.right_image_sub = self.create_subscription(
            msg_type=CompressedImage,
            topic='right_image',
            callback=self.right_image_cb,
            qos_profile=1,
        )

    def left_image_cb(self, msg: CompressedImage):
        data = msg.data.tobytes()
        buff = np.frombuffer(data, dtype=np.uint8)
        img = cv.imdecode(buff, cv.IMREAD_COLOR)
        cv.imshow('left_image', img)
        cv.waitKey(1)

    def right_image_cb(self, msg: CompressedImage):
        data = msg.data.tobytes()
        buff = np.frombuffer(data, dtype=np.uint8)
        img = cv.imdecode(buff, cv.IMREAD_COLOR)
        cv.imshow('right_image', img)
        cv.waitKey(1)


def main():
    """Run main loop."""
    rclpy.init()

    image_viewer = ImageViewer()
    rclpy.spin(image_viewer)

    image_viewer.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
