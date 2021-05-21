"""ROS node asserving both cameras.

cameras will focus on the best global image
In order to restart the focus algoritm because of zoom or environment changing
press "r" key
"""

import rclpy
from rclpy.node import Node

import numpy as np
from pynput import keyboard

from sensor_msgs.msg._compressed_image import CompressedImage
from reachy_msgs.srv import SetCameraFocusZoom, GetCameraFocusZoom
from reachy_msgs.srv import Set2CamerasFocus

import cv2 as cv
from cv_bridge import CvBridge

import time
import threading

from functools import partial


def move_to(min_pos, max_pos, pos, step):
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


def set_poses(zoom):
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
                'compressed_img': CompressedImage(),
            },
            'right_eye': {
                'pos': 0,
                'final_pos': 0,
                'init': True,
                'current_zoom': -1,
                'compressed_img': CompressedImage(),
            },
        }

        self.pos = {
            'left_eye': 0,
            'right_eye': 0,
        }

        self.final_pos = {
            'left_eye': -1,
            'right_eye': -1,
        }

        self.current_zoom = {
            'left_eye': -1,
            'right_eye': -1,
        }

        self.start = True
        # self.zoom = -1
        # self.last_zoom = -1

        self.bridge = CvBridge()

        self.camera_subscriber_left = self.create_subscription(
            CompressedImage,
            'left_image',
            partial(self.on_image_update, side='left'),
            1,
        )

        self.camera_subscriber_right = self.create_subscription(
            CompressedImage,
            'right_image',
            partial(self.on_image_update, side='right'),
            1,
        )

        self.set_camera_focus_zoom_client = self.create_client(
            SetCameraFocusZoom,
            'set_camera_focus_zoom')
        while not self.set_camera_focus_zoom_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service set_camera_focus_zoom_client not available, waiting again...')
        self.req = SetCameraFocusZoom.Request()

        self.set_focus_2_cameras_client = self.create_client(
            Set2CamerasFocus,
            'set_2_cameras_focus')
        while not self.set_focus_2_cameras_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service set_focus_2_cameras_client not available, waiting again...')
        self.req_focus_2_cam = Set2CamerasFocus.Request()

        self.get_zoom_focus_client = self.create_client(
            GetCameraFocusZoom,
            'get_camera_focus_zoom')
        while not self.get_zoom_focus_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service get_zoom_focus_client not available, waiting again...')
        self.req_zoom_focus = GetCameraFocusZoom.Request()

        self.keyboard_listener = keyboard.Listener(on_press=self.on_press)
        self.keyboard_listener.start()

        self.right_eye_thread = threading.Thread(
            target=self.focusing_algorithm,
            args=('left_eye',),
            daemon=True)
        self.left_eye_thread = threading.Thread(
            target=self.focusing_algorithm,
            args=('right_eye',),
            daemon=True)
        self.e_init = threading.Event()
        self.e_end = threading.Event()

        self.right_eye_thread.start()
        self.left_eye_thread.start()

    def canny_sharpness_function(self, compressed_img):
        """Return the shaprness of im through canny edge dectection algorithm.

        Args:
            im: Black an white image used in canny edge detection algorithm
        """
        im = self.bridge.compressed_imgmsg_to_cv2(compressed_img)
        im = cv.Canny(cv.cvtColor(im, cv.COLOR_BGR2GRAY), 50, 100)
        im_sum = cv.integral(im)
        return im_sum[-1][-1]/(im.shape[0]*im.shape[1])

    def on_image_update(self, msg, side):
        """Get data from image. Callback for "/'side'_image "subscriber."""
        # self.eyes_info[side+'_eye']['img'] = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.eyes_info[side+'_eye']['compressed_img'] = msg

    def send_request_set_focus_zoom(self, name, zoom, focus):
        """Send request through "set_camera_focus_zoom_client" client.

        Args:
            name: camera name, can be 'left_eye' or 'right_eye'
            zoom: integer zoom desired value
            focus: integer focus desired value

        """
        self.req.name = name
        self.req.zoom = zoom
        self.req.focus = focus
        self.future = self.set_camera_focus_zoom_client.call_async(self.req)

    def send_request_set_focus_2_cam(self, left_focus, right_focus):
        """Send request through "set_focus_2_cameras_client" client.

        This client ask cameras to go to "ref_focus" and "right_focus respectively at once"

        Args:
            left_focus: left camera integer focus desired value
            right_focus: right camera integer focus desired value
        """
        self.req_focus_2_cam.left_focus = left_focus
        self.req_focus_2_cam.right_focus = right_focus
        self.future_focus_2_cam = self.set_focus_2_cameras_client.call_async(self.req_focus_2_cam)

    def send_request_get_focus_zoom(self, name):
        """Send request through "get_zoom_focus_client" client.

        Args :
            name : camera name, can be 'left_eye' or 'right_eye'
        """
        self.req_zoom_focus.name = name
        self.future_zoom_focus = self.get_zoom_focus_client.call_async(self.req_zoom_focus)

    def focusing_algorithm(self, eye):
        """Endless loop which handle the focus of one camera refered as "eye".

        Cameras focus motors start and stop at once
        focussing choice are independent

        Args:
            eye: camera name, can be 'left_eye' or 'right_eye'
        """
        max_res = 0  # Best canny sharpness function result obtained
        p_max = 0  # focus position link to max_res
        min_pos = 0  # minimal focus position reachable
        max_pos = 0  # maximal focus position reachable
        low_thresh = 0  # lower noise tolerance threshold
        up_thresh = 0  # upper noise tolerance threshold
        step = 1  # moving step
        noise = 0.4

        self.eyes_info[eye]['init'] = True
        first = True  # True means first iteration
        stop = 0
        zoom = -1
        last_zoom = -1
        time.sleep(1)

        while(1):
            if self.start:
                res = self.canny_sharpness_function(self.eyes_info[eye]['compressed_img'])

                if self.eyes_info[eye]['init']:
                    # while (self.eyes_info['left_eye']['current_zoom'] == -1 or self.eyes_info['right_eye']['current_zoom'] == -1):
                    while(self.current_zoom["left_eye"] == -1 or self.current_zoom["right_eye"] == -1):
                        self.send_request_get_focus_zoom(eye)
                        self.test_response(self.future_zoom_focus)
                        try:
                            # self.eyes_info[eye]['current_zoom'] = self.future_zoom_focus.result().zoom
                            self.current_zoom[eye] = self.future_zoom_focus.result().zoom
                        except Exception:
                            pass

                    # if self.eyes_info['left_eye']['current_zoom'] == self.eyes_info['right_eye']['current_zoom']:
                    if self.current_zoom["left_eye"] == self.current_zoom["right_eye"]:
                        zoom = self.current_zoom["left_eye"]
                        # zoom = self.eyes_info['left_eye']['current_zoom']

                    if zoom < 100:
                        noise = 5

                    first = True
                    stop = 0
                    min_pos, max_pos = set_poses(zoom)
                    self.pos[eye] = min_pos
                    # self.eyes_info[eye]['pos'] = min_pos

                    max_res = 0
                    step = 1
                    self.eyes_info[eye]['init'] = False

                    if (eye == "left_eye" and self.eyes_info['right_eye']['init'] is False) or (eye == 'right_eye' and self.eyes_info['left_eye'] is False):
                    # if (eye == "left_eye" and self.init["right_eye"] is False) or (eye == "right_eye" and self.init["left_eye"] is False):
                        self.send_request_set_focus_2_cam(min_pos, min_pos)
                        if last_zoom != zoom:
                            time.sleep(4)  # leave enough time in case of zoom change
                            last_zoom = zoom
                        else:
                            time.sleep(2)
                        self.e_init.set()
                        self.e_init.clear()
                    else:
                        self.e_init.wait()

                elif stop == 0:
                    if res > max_res:
                        max_res = res
                        p_max = self.pos[eye]
                        # p_max = self.eyes_info[eye]['pos']

                    if first:
                        first = False
                        low_thresh = res - noise
                        up_thresh = res + noise

                        # self.eyes_info[eye]['pos'] = move_to(min_pos, max_pos, self.eyes_info[eye]['pos'], step)
                        self.pos[eye] = move_to(min_pos, max_pos, self.pos[eye], step)
                    # elif res < low_thresh or self.eyes_info[eye]['pos'] == max_pos:
                    elif res < low_thresh or self.pos[eye] == max_pos:
                        # self.eyes_info[eye]['final_pos'] = p_max
                        self.final_pos[eye] = p_max
                        # if (eye == 'left_eye' and self.eyes_info['right_eye']['final_pos'] > -1) or (eye == 'right_eye' and self.eyes_info['left_eye']['final_pos'] > -1):
                        if (eye == "left_eye" and self.final_pos["right_eye"] > -1) or (eye == "right_eye" and self.final_pos["left_eye"] > -1):
                            stop = 1

                            # temp_left = move_to(
                            #     min_pos,
                            #     max_pos,
                            #     self.eyes_info['left_eye']['final_pos'],
                            #     -30,
                            # )
                            # temp_right = move_to(
                            #     min_pos,
                            #     max_pos,
                            #     self.eyes_info['right_eye']['final_pos'],
                            #     -30,
                            # )
                            temp_left = move_to(min_pos, max_pos,
                                                self.final_pos["left_eye"],
                                                -30)
                            temp_right = move_to(min_pos, max_pos,
                                                 self.final_pos["right_eye"],
                                                 -30)
                            self.send_request_set_focus_2_cam(temp_left,
                                                              temp_right)
                            time.sleep(0.5)
                            self.send_request_set_focus_2_cam(
                                self.final_pos["left_eye"],
                                self.final_pos["right_eye"])
                            # self.send_request_set_focus_2_cam(
                            #     self.eyes_info['left_eye']['final_pos'],
                            #     self.eyes_info['right_eye']['final_pos'],
                            # )
                            time.sleep(0.5)
                            self.e_end.set()

                            # self.eyes_info[eye]['pos'] = self.eyes_info[eye]['final_pos']
                            # self.eyes_info[eye]['final_pos'] = -1
                            self.pos[eye] = self.final_pos[eye]
                            self.final_pos[eye] = -1
                            self.e_end.clear()
                        else:
                            self.e_end.wait()

                            # self.eyes_info[eye]['pos'] = self.eyes_info['final_pos']
                            self.pos[eye] = self.final_pos[eye]
                            self.final_pos[eye] = -1
                            # self.eyes_info['final_pos'] = -1

                            stop = 1
                        self.start = False

                    elif res > up_thresh:
                        low_thresh = res - noise
                        up_thresh = res + noise
                        step = 1
                        # self.eyes_info[eye]['pos'] = move_to(min_pos, max_pos, self.eyes_info[eye]['pos'], step)
                        self.pos[eye] = move_to(min_pos,
                                                max_pos,
                                                self.pos[eye],
                                                step)

                    else:
                        if step == 1:
                            step = 5
                        # self.eyes_info[eye]['pos'] = move_to(min_pos, max_pos, self.eyes_info[eye]['pos'], step)
                        self.pos[eye] = move_to(min_pos,
                                                max_pos,
                                                self.pos[eye],
                                                step)

                    # self.send_request_get_focus_zoom(
                    #     eye,
                    #     zoom,
                    #     self.eyes_info[eye]['pos'],
                    # )
                    self.send_request_set_focus_zoom(eye,
                                                     zoom,
                                                     self.pos[eye])
                    time.sleep(0.15)

            else:
                time.sleep(0.04)

    def test_response(self, future):
        """Wait for service answer.

        Args:
            future : returned value by client asynchronous call
        """
        while(rclpy.ok()):
            if future.done():
                try:
                    _ = future.result()
                except Exception as e:
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                break
            time.sleep(0.001)

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
            # self.eyes_info['left_eye']['current_zoom'] = -1
            # self.eyes_info['right_eye']['current_zoom'] = -1

            self.current_zoom['left_eye'] = -1
            self.current_zoom['right_eye'] = -1
            if self.start is False:
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
