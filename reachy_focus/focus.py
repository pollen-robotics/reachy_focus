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


def Canny_Sharpness_function(im):
    """Return the shaprness of im through canny edge dectection algorithm"""
    im = cv.Canny(im, 50, 100)
    sum = cv.integral(im)
    return sum[-1][-1]/(im.shape[0]*im.shape[1])


def move_to(pos_min, pos_max, pos, pas):
    """Return the next position to reach regarding range limitations

    pos_min and pos_max are range limitations
    pos is the curent position of the stepper motor
    pas is the step between the current position and the next desired, can be positive as negative value
    """
    if pos_min < pos + pas < pos_max:
        pos += pas
    elif pos + pas >= pos_max:
        pos = pos_max
    elif pos + pas <= pos_min:
        pos = pos_min
    return pos


def set_poses(zoom):
    """return range limitation regarding current zoom position"""
    pos_min = int(500 - (np.math.exp(0.01*zoom)+25)*5)
    pos_max = int(500 - (np.math.exp(0.05*zoom/6)+5)*5)
    if pos_min < 0:
        pos_min = 0
    if pos_max > 500:
        pos_max = 500
    return pos_min, pos_max


class CameraFocus(Node):
    """The CameraFocus class handle the focus of both reachy cameras in real time

    It hold :
        - recovery of cameras images
        - control of focus cameras motors
    """

    def __init__(self):
        """Setup variables shared between threads, publishers and clients"""
        super().__init__('camera_focus')

        self.pos = {
            'left_eye': 0,
            'right_eye': 0,
        }

        self.final_pos = {
            'left_eye': -1,
            'right_eye': -1,
        }

        self.Init = {
            'left_eye': True,
            'right_eye': True,
        }

        self.current_zoom = {
            'left_eye': -1,
            'right_eye': -1,
        }

        self.img = {
            'left_eye': CompressedImage(),
            'right_eye': CompressedImage(),
        }

        self.Start = True
        self.zoom = -1
        self.bruit = 0.4

        self.bridge = CvBridge()

        self.camera_subscriber_left = self.create_subscription(
            CompressedImage, 'left_image',
            self.listener_callback_left,
            10)

        self.camera_subscriber_right = self.create_subscription(
            CompressedImage, 'right_image',
            self.listener_callback_right,
            10)

        self.set_camera_focus_zoom_client = self.create_client(SetCameraFocusZoom,
                                                               'set_camera_focus_zoom')
        while not self.set_camera_focus_zoom_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service set_camera_focus_zoom_client not available, waiting again...')
        self.req = SetCameraFocusZoom.Request()

        self.set_focus_2_cameras_client = self.create_client(Set2CamerasFocus,
                                                             'set_2_cameras_focus')
        while not self.set_focus_2_cameras_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service set_focus_2_cameras_client not available, waiting again...')
        self.req_focus_2_cam = Set2CamerasFocus.Request()

        self.get_zoom_focus_client = self.create_client(GetCameraFocusZoom,
                                                        'get_camera_focus_zoom')
        while not self.get_zoom_focus_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service get_zoom_focus_client not available, waiting again...')
        self.req_zoom_focus = GetCameraFocusZoom.Request()

        self.keyboard_listener = keyboard.Listener(on_press=self.on_press)
        self.keyboard_listener.start()

        self.right_eye_thread = threading.Thread(target=self.asserv, args=('right_eye', 'left_image'), daemon=True)
        self.left_eye_thread = threading.Thread(target=self.asserv, args=('left_eye', 'right_image'), daemon=True)
        self.e_init = threading.Event()
        self.e_end = threading.Event()

        self.right_eye_thread.start()
        self.left_eye_thread.start()

    def listener_callback_left(self, msg):
        """Save last left_image catched"""
        self.img['left_image'] = msg

    def listener_callback_right(self, msg):
        """Save last right_image catched"""
        self.img['right_image'] = msg

    def send_request_set_focus_zoom(self, name, zoom, focus):
        """Send request through "set_camera_focus_zoom_client" client

        This client ask "name" eye to go to desired "zoom" and "focus"
        """
        self.req.name = name
        self.req.zoom = zoom
        self.req.focus = focus
        self.future = self.set_camera_focus_zoom_client.call_async(self.req)

    def send_request_set_focus_2_cam(self, left_focus, right_focus):
        """Send request through "set_focus_2_cameras_client" client

        This client ask cameras to go to "ref_focus" and "right_focus respectively at once"
        """
        self.req_focus_2_cam.left_focus = left_focus
        self.req_focus_2_cam.right_focus = right_focus
        self.future_focus_2_cam = self.set_focus_2_cameras_client.call_async(self.req_focus_2_cam)

    def send_request_get_focus_zoom(self, name):
        """Send request through "get_zoom_focus_client" client

        This client ask "name" eye its current zoom
        """
        self.req_zoom_focus.name = name
        self.future_zoom_focus = self.get_zoom_focus_client.call_async(self.req_zoom_focus)

    def asserv(self, eye, im):
        """Endless loop which handle the focus of one camera refered as "eye"

        Cameras focus motors start and stop at once
        focussing choice are independent
        """
        res_max = 0  # meilleur résultat de canny obtenu
        p_max = 0  # posistion du focus associée à res_max
        pos_min = 0  # position minimale accessible du focus
        pos_max = 0  # position maximale accessible du focus
        borne_inf = 0  # borne haute de tolerance du bruit
        borne_sup = 0  # borne basse de tolerance du bruit
        pas = 1  # pas d'avance
        canny = []  # liste de stockage des résultats donnés par canny
        poses = []  # liste de stockage des positions associées à canny[]
        j = 0  # variable de numérotation des diagrammes

        self.Init[eye] = True
        First = True  # Flag à True s'il stagit de la première itération
        Stop = 0  # 0 = pas stop, 1 = 1er passage en stop, 2 = plus de 1 passage en stop
        time.sleep(1)

        while(1):
            if self.Start:
                img = self.bridge.compressed_imgmsg_to_cv2(self.img[im])
                imgBW = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
                res = Canny_Sharpness_function(imgBW)
                # print(eye+"_res = "+str(res))
                # print(eye+"_res max = "+str(res_max))
                # print(eye+"_pos = " + str(self.pos[eye]))

                if self.Init[eye] is True:
                    while(self.current_zoom["left_eye"] == -1 or self.current_zoom["right_eye"] == -1):
                        self.send_request_get_focus_zoom(eye)
                        self.test_response(self.future_zoom_focus)
                        try:
                            self.current_zoom[eye] = self.future_zoom_focus.result().zoom
                        except Exception as e:
                            pass

                    if self.current_zoom["left_eye"] == self.current_zoom["right_eye"]:
                        self.zoom = self.current_zoom["left_eye"]
                    # print("zoom " + str(self.zoom))

                    if self.zoom < 100:
                        self.bruit = 5

                    First = True
                    Stop = 0
                    pos_min, pos_max = set_poses(self.zoom)
                    self.pos[eye] = pos_min
                    res_max = 0
                    pas = 1
                    self.Init[eye] = False

                    if (eye == "left_eye" and self.Init["right_eye"] is False) or (eye == "right_eye" and self.Init["left_eye"] is False):
                        self.send_request_set_focus_2_cam(pos_min, pos_min)  # déplacement des moteurs aux positions de départs
                        time.sleep(4)  # laisser suffisemment de temps au cas où les zooms ont beaucoup changés
                        # self.test_response(self.future)
                        self.e_init.set()
                        self.e_init.clear()
                    else:
                        self.e_init.wait()

                elif Stop == 0:
                    # cv.imwrite("src/reachy_focus/rec/"+str(eye)+"_"+str(self.pos[eye])+".png", img)
                    # print("stop = 0")
                    if res > res_max:
                        # print("res > res_max")
                        res_max = res
                        p_max = self.pos[eye]

                    if First is True:
                        # print("First = True")
                        j += 1
                        tp1 = time.time()
                        First = False
                        borne_inf = res-self.bruit
                        borne_sup = res+self.bruit
                        # print ("b_min = "+ str(borne_inf)+"b_max = "+str(borne_sup))
                        self.pos[eye] = move_to(pos_min, pos_max, self.pos[eye], pas)
                    elif res < borne_inf or self.pos[eye] == pos_max:
                        # print ("res < borne_inf, p_max = " + str(p_max))
                        self.final_pos[eye] = p_max
                        tp2 = time.time()
                        print("!!!!!!!!!!!!time = " + str(tp2-tp1))
                        if (eye == "left_eye" and self.final_pos["right_eye"] > -1) or (eye == "right_eye" and self.final_pos["left_eye"] > -1):
                            Stop = 1
                            temp_left = move_to(pos_min, pos_max, self.final_pos["left_eye"], -30)
                            temp_right = move_to(pos_min, pos_max, self.final_pos["right_eye"], -30)
                            self.send_request_set_focus_2_cam(temp_left, temp_right)
                            time.sleep(0.5)
                            # print(str(eye) + ": retour en arr")
                            self.send_request_set_focus_2_cam(self.final_pos["left_eye"], self.final_pos["right_eye"])
                            time.sleep(0.5)
                            print(str(eye) + ": pos max atteind, right_eye = " + str(self.final_pos["right_eye"])+" left_eye = " + str(self.final_pos["left_eye"]))
                            self.e_end.set()
                            # self.test_response(self.future)
                            self.pos[eye] = self.final_pos[eye]
                            self.final_pos[eye] = -1
                            self.e_end.clear()
                        else:
                            # print(str(eye) "+ " attend")
                            self.e_end.wait()
                            self.pos[eye] = self.final_pos[eye]
                            self.final_pos[eye] = -1
                            Stop = 1
                        self.Start = False

                    elif res > borne_sup:
                        # print("borne dépassée")
                        borne_inf = res-self.bruit
                        borne_sup = res+self.bruit
                        # print("b_min = " + str(borne_inf)+"b_max = " + str(borne_sup))
                        pas = 1
                        self.pos[eye] = move_to(pos_min, pos_max, self.pos[eye], pas)

                    else:
                        # print("dans les bornes")
                        if pas == 1:
                            pas = 5
                        self.pos[eye] = move_to(pos_min, pos_max, self.pos[eye], pas)

                    self.send_request_set_focus_zoom(eye, self.zoom, self.pos[eye])
                    # print(str(eye) + "pos send = " + str(self.pos[eye]))
                    time.sleep(0.15)

            else:
                time.sleep(0.04)

    def test_response(self, future):
        """Wait for service answer """
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
        """Callback function call after key press event

        "r" key: restart the focus algorithm
        "s" key: Allow to start/stop the focus algorithm"""
        if str(key) == "'r'":
            print("restart the sequence")
            self.Init['left_eye'] = True
            self.Init['right_eye'] = True
            self.current_zoom['left_eye'] = -1
            self.current_zoom['right_eye'] = -1
            if self.Start is False:
                self.Start = True

        if str(key) == "'s'":
            if self.Start is True:
                self.Start = False
                print("Stop")
            else:
                self.Start = True
                print("Start")


def main(args=None):
    """Main function

    Create and launch CameraFocus Node
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
