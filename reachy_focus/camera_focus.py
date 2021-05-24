"""ROS node asserving both cameras.

cameras will focus on the best global image
In order to restart the focus algoritm because of zoom or environment changing
press "r" key
"""

from typing import Dict
import rclpy
from rclpy.node import Node

import numpy as np
from pynput import keyboard

from sensor_msgs.msg._compressed_image import CompressedImage
from reachy_msgs.msg import ZoomCommand
from reachy_msgs.srv import GetCameraZoomFocus, SetCameraZoomFocus

import cv2 as cv
from cv_bridge import CvBridge

import time
import threading
from functools import partial


def compute_next_pose(min_pos, max_pos, pos, step):
    """Return the next position to reach regarding range limitations.

    Args:
        max_pos: upper position limitation
        min_pos: lower position limitation
        pos: curent position of the stepper motor
        step:step between the current position and the next desired,
        can be positive as negative value
    """
    if min_pos < pos + step < max_pos:
        pos += step
    elif pos + step >= max_pos:
        pos = max_pos
    elif pos + step <= min_pos:
        pos = min_pos
    return pos


def compute_poses_maxima(zoom):
    """Return range limitation regarding current zoom position.

    Args:
        zoom: current zoom value
    """
    min_pos = max(int(500 - (np.math.exp(0.01*zoom)+25)*5), 0)
    max_pos = min(int(500 - (np.math.exp(0.05*zoom/6)+5)*5), 500)
    return min_pos, max_pos


class CameraFocus(Node):
    """The CameraFocus class handle the focus of both reachy cameras in real time.

    It hold :
        - recovery of cameras images
        - control of focus cameras motors
    """

    def __init__(self):
        """Set-up variables shared between threads, publishers and clients."""
        super().__init__('camera_focus')

        self.eyes_info = {
            'left_eye': {
                'pos': 0,
                'final_pos': 0,
                'init': True,
                'current_zoom': -1,
                'compressed_img': None,
            },
            'right_eye': {
                'pos': 0,
                'final_pos': 0,
                'init': True,
                'current_zoom': -1,
                'compressed_img': None,
            },
        }

        self.start = True

        self.bridge = CvBridge()

        self.camera_subscriber_left = self.create_subscription(
            CompressedImage, 'left_image',
            partial(self.on_image_update, side='left'),
            1,
            )

        self.camera_subscriber_right = self.create_subscription(
            CompressedImage, 'right_image',
            partial(self.on_image_update, side='right'),
            1,
            )

        self.set_camera_zoom_focus_client = self.create_client(
            SetCameraZoomFocus,
            'set_camera_zoom_focus',
        )

        self.get_camera_zoom_focus_client = self.create_client(
            GetCameraZoomFocus,
            'get_camera_zoom_focus',
        )

        time.sleep(5.0)

        self.keyboard_listener = keyboard.Listener(on_press=self.on_press)
        self.keyboard_listener.start()

        self.right_eye_thread = threading.Thread(
            target=self.focusing_algorithm,
            args=('right_eye',),
            daemon=True)
        self.left_eye_thread = threading.Thread(
            target=self.focusing_algorithm,
            args=('left_eye',),
            daemon=True)
        self.e_init = threading.Event()
        self.e_end = threading.Event()

        self.right_eye_thread.start()
        self.left_eye_thread.start()

    def _wait_for(self, future):
        for _ in range(10000):
            if future.done():
                return future.result()
            time.sleep(0.001)

    def canny_sharpness_function(self, im):
        """Return the shaprness of im through canny edge dectection algorithm.

        Args:
            im: image used in canny edge detection algorithm
        """
        im = self.bridge.compressed_imgmsg_to_cv2(im)
        im = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
        im = cv.Canny(im, 50, 100)
        im_sum = cv.integral(im)
        return im_sum[-1][-1]/(im.shape[0]*im.shape[1])

    def on_image_update(self, msg, side):
        """Get data from image. Callback for "/'side'_image "subscriber."""
        self.eyes_info[side+'_eye']['compressed_img'] = msg

    def send_request_set_camera_zoom_focus(self, command: Dict):
        req = SetCameraZoomFocus.Request()

        for side, cmd in command.items():
            for cmd_name, value in cmd.items():
                zoom_cmd_msg = ZoomCommand()
                zoom_cmd_msg.flag = True
                zoom_cmd_msg.value = value
                setattr(req, side+'_'+cmd_name, zoom_cmd_msg)
        result = self._wait_for(self.set_camera_zoom_focus_client.call_async(req))
        return result

    def send_request_get_camera_zoom_focus(self):
        req = GetCameraZoomFocus.Request()
        result = self._wait_for(self.get_camera_zoom_focus_client.call_async(req))
        return result

    def focusing_algorithm(self, eye):
        """Endless loop which handle the focus of one camera refered as "eye".

        Cameras focus motors start and stop at once
        focussing choice are independent

        Args:
            eye: camera name, can be 'left_eye' or 'right_eye'
            im: image position, can be 'left_image' or 'right_image'
        """
        max_res = 0  # Best canny sharpness function result obtained
        p_max = 0  # focus position link to max_res
        min_pos = 0  # minimal focus position reachable
        max_pos = 0  # maximal focus position reachable
        low_thresh = 0  # lower noise tolerance threshold
        up_thresh = 0  # upper noise tolerance threshold
        step = 1  # moving step

        self.eyes_info[eye]['init'] = True
        first = True  # True means first iteration
        stop = 0
        zoom  = self.eyes_info[eye]['current_zoom']
        noise = 0.4

        eye_side = eye.split('_')[0]
        time.sleep(1)

        while(1):
            if self.start:
                res = self.canny_sharpness_function(self.eyes_info[eye]['compressed_img'])

                if self.eyes_info[eye]['init']:
                    while self.eyes_info[eye]['current_zoom'] == -1:
                        self.eyes_info[eye]['current_zoom'] = getattr(self.send_request_get_camera_zoom_focus(), eye_side+'_zoom')

                    zoom = self.eyes_info[eye]['current_zoom']

                    if zoom < 100:
                        noise = 5

                    first = True
                    stop = 0
                    min_pos, max_pos = compute_poses_maxima(zoom)
                    self.eyes_info[eye]['pos'] = min_pos
                    max_res = 0
                    step = 1
                    self.eyes_info[eye]['init'] = False

                    self.send_request_set_camera_zoom_focus({eye_side: {'focus': min_pos}})
                    time.sleep(2)

                elif stop == 0:
                    if res > max_res:
                        max_res = res
                        p_max = self.eyes_info[eye]['pos']

                    if first:
                        first = False
                        low_thresh = res - noise
                        up_thresh = res + noise
                        self.eyes_info[eye]['pos'] = compute_next_pose(min_pos, max_pos, self.eyes_info[eye]['pos'], step)
                    elif res < low_thresh or self.eyes_info[eye]['pos'] == max_pos:
                        self.eyes_info[eye]['final_pos'] = p_max
                        if (eye == 'left_eye' and self.eyes_info['right_eye']['final_pos'] > - 1) or (eye == 'right_eye' and self.eyes_info['left_eye']['final_pos'] > -1):
                            stop = 1
                            temp_left = compute_next_pose(
                                                    min_pos,
                                                    max_pos,
                                                    self.eyes_info['left_eye']['final_pos'],
                                                    -30,
                                                    )
                            temp_right = compute_next_pose(
                                                    min_pos,
                                                    max_pos,
                                                    self.eyes_info['right_eye']['final_pos'],
                                                    -30,
                                                    )
                            self.send_request_set_camera_zoom_focus({'left': {'focus': temp_left}, 'right': {'focus': temp_right}})

                            time.sleep(0.5)
                            self.send_request_set_camera_zoom_focus({'left': {'focus': self.eyes_info["left_eye"]['final_pos']}, 'right': {'focus': self.eyes_info["right_eye"]['final_pos']}})
                            time.sleep(0.5)
                            self.e_end.set()
                            
                            self.eyes_info[eye]['pos'] = self.eyes_info[eye]['final_pos']
                            self.eyes_info[eye]['final_pos'] = -1
                            self.e_end.clear()
                        else:
                            self.e_end.wait()
                            self.eyes_info[eye]['pos'] = self.eyes_info[eye]['final_pos']
                            self.eyes_info[eye]['final_pos'] = -1
                            stop = 1
                        self.start = False

                    elif res > up_thresh:
                        low_thresh = res - noise
                        up_thresh = res + noise
                        step = 1
                        self.eyes_info[eye]['pos'] = compute_next_pose(
                                                    min_pos,
                                                    max_pos,
                                                    self.eyes_info[eye]['pos'],
                                                    step,
                                                    )

                    else:
                        if step == 1:
                            step = 5
                        self.eyes_info[eye]['pos'] = compute_next_pose(
                                                    min_pos,
                                                    max_pos,
                                                    self.eyes_info[eye]['pos'],
                                                    step,
                                                    )
                    self.send_request_set_camera_zoom_focus({eye.split('_')[0]: {'zoom': zoom, 'focus': self.eyes_info[eye]['pos']}})
                    time.sleep(0.15)

            else:
                time.sleep(0.04)


    def on_press(self, key):
        """Call after key press event.

        "r" key: restart the focus algorithm
        "s" key: Allow to start/stop the focus algorithm

        Args:
            key: key press id
        """
        if str(key) == "'r'":
            print("restart the sequence")
            self.eyes_info['left_eye']['init'] = True
            self.eyes_info['right_eye']['init'] = True            
            self.eyes_info['left_eye']['current_zoom'] = -1
            self.eyes_info['right_eye']['current_zoom'] = -1
            if not self.start:
                self.start = True

        if str(key) == "'s'":
            if self.start is True:
                self.start = False
                print("stop")
            else:
                self.start = True
                print("start")


def main(args=None):
    """Create and launch CameraFocus Node.

    If ctrl+c is pressed node is destroyed
    """
    rclpy.init(args=args)

    camera_focus = CameraFocus()

    try:
        rclpy.spin(camera_focus)
    except KeyboardInterrupt:

        camera_focus.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
