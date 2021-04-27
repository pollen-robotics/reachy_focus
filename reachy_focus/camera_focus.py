"""ROS node asserving both cameras ."""

import rclpy
import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
import threading
import time
from rclpy.node import Node
from pynput import keyboard

from sensor_msgs.msg._compressed_image import CompressedImage
from reachy_msgs.srv import SetCameraFocusZoom, GetCameraFocusZoom
from reachy_msgs.srv import Set2CamerasFocus


from cv_bridge import CvBridge


# class CameraSubscriber(Node):

#     def __init__(self):
#         super().__init__('camera_subscriber')

#         # self.image_left = CompressedImage()
#         self.bridge = CvBridge()

#         self.camera_subscriber_left = self.create_subscription(
#             CompressedImage, 'left_image',
#             self.listener_callback,
#             10)
#         self.camera_subscriber_left  # prevent unused variable warning

#     def listener_callback(self, msg):
#         global img
#         img = self.bridge.compressed_imgmsg_to_cv2(msg)
#         # print(img)

# class CameraZoomClient(Node):

#     def __init__(self):
#         super().__init__('camera_zoom_client')
#         self.set_camera_focus_zoom_client = self.create_client(SetCameraFocusZoom,
#             'set_camera_focus_zoom')
#         while not self.set_camera_focus_zoom_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('service not available, waiting again...')
#         self.req = SetCameraFocusZoom.Request()

#     def send_request(self, name, zoom, focus):
#         self.req.name = name
#         self.req.zoom = zoom
#         self.req.focus = focus
#         self.future = self.set_camera_focus_zoom_client.call_async(self.req)


def Canny_Sharpness_function(im):
    im = cv.Canny(im, 50, 100)
    sum = cv.integral(im)  # calcul la somme cumulée de touts les pixel dans l'image
    # print("sum canny = " + str(sum[-1][-1]) )
    return sum[-1][-1]/(im.shape[0]*im.shape[1])


def move_to(pos_min, pos_max, pos, pas):
    if pos_min < pos + pas < pos_max:
        pos += pas
    elif pos + pas >= pos_max:
        pos = pos_max
    elif pos + pas <= pos_min:
        pos = pos_min
    return pos


def set_poses(zoom):
    pos_min = int(500 - (np.math.exp(0.01*zoom)+25)*5)
    pos_max = int(500 - (np.math.exp(0.05*zoom/6)+5)*5)
    ######Saturation des positions ateignables#####
    if pos_min < 0:
        pos_min = 0
    if pos_max > 500:
        pos_max = 500
    print(str(pos_min) + ", "+ str(pos_max))
    return pos_min, pos_max


class CameraFocus(Node):
    def __init__(self):
        super().__init__('camera_focus')

        self.zoom = 50
        self.bruit = 0.4
        self.pos = {
            'left_eye': 0,
            'right_eye': 0,
        }

        self.Init = {
            'left_eye': True,
            'right_eye': True,
        }

        self.final_pos = {
            'left_eye': -1,
            'right_eye': -1,
        }

        self.Start = True
        self.Move = False
        self.Points = []
        self.k = 0

        self.bridge = CvBridge()
        self.img = {
            'left_eye': CompressedImage(),
            'right_eye': CompressedImage(),
        }

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

        self.t = threading.Thread(target=self.asserv, args=('right_eye', 'left_image'), daemon=True)
        self.t2 = threading.Thread(target=self.asserv, args=('left_eye', 'left_image'), daemon=True)
        self.e_init = threading.Event()
        self.e_end = threading.Event()

        self.t.start()
        self.t2.start()

        # self._log = {'left': [], 'right': []}

    def listener_callback_left(self, msg):
        # print ("in listener callback")
        # print(type(msg))
        self.img['left_image'] = msg
        # self.img['left_image'] = self.bridge.compressed_imgmsg_to_cv2(msg)
        # print("listen left", time.time())
        # self.k += 1
        # cv.imshow("video",self.img)
        # print(type(self.img))
        # print(self.img)


    def listener_callback_right(self, msg):
        self.img['right_image'] = msg
        # self.img['right_image'] = self.bridge.compressed_imgmsg_to_cv2(msg)
        # print("listen right", time.time())

        # print("listen right")

    def send_request_set_focus_zoom(self, name, zoom, focus):
        self.req.name = name
        self.req.zoom = zoom
        self.req.focus = focus
        self.future = self.set_camera_focus_zoom_client.call_async(self.req)

    def send_request_set_focus_2_cam(self, left_focus, right_focus):
        self.req_focus_2_cam.left_focus = left_focus
        self.req_focus_2_cam.right_focus = right_focus
        self.future_focus_2_cam = self.set_focus_2_cameras_client.call_async(self.req_focus_2_cam)

    def send_request_get_focus_zoom(self, name):
        self.req_zoom_focus.name = name
        self.future_zoom_focus = self.get_zoom_focus_client.call_async(self.req_zoom_focus)
   
    def asserv(self, eye, im):
        res_max = 0  # meilleur résultat de canny obtenu
        p_max = 0  # posistion du focus associée à res_max
        pos_min = 0  # position minimale accessible du focus
        pos_max = 0  # position maximale accessible du focus
        borne_inf = 0  # borne haute de tolerance du bruit
        borne_sup = 0  # borne basse de tolerance du bruit
        pas = 1  # pas d'avance
        canny = []  # liste de stockage des résultats donnés par canny
        poses = []  # liste de stockage des positions associées à canny[]
        seuil = 0.5  # seuil d'interet, en dessous de cette valeur il ne peut pas y avoir de pic
        k = -1  # variable de numérotation des images
        j = 0  # variable de numérotation des diagrammes
        sem = 0

        First = True  # Flag à True s'il stagit de la première itération
        Stop = 0  # 0 = pas stop, 1 = 1er passage en stop, 2 = plus de 1 passage en stop
        time.sleep(1)
        img = self.bridge.compressed_imgmsg_to_cv2(self.img[im])
        img_prec_bw = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        while(1):
            # print("asserv " + str(eye) + " " + str(j))
            # j += 1
            self.send_request_get_focus_zoom(eye)
            img = self.bridge.compressed_imgmsg_to_cv2(self.img[im])
            if len(self.Points) == 2:
                roi = img[self.Points[0][1]:self.Points[1][1], self.Points[0][0]:self.Points[1][0]]
                imgBW = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
            else:
                imgBW = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

            if self.Start is True:
                # # tp1 = time.time()
                # # img_bw = cv.cvtColor(img[im], cv.COLOR_BGR2GRAY)
                # t0 = time.time()
                diff = cv.subtract(img_prec_bw, imgBW)
                ret, diff = cv.threshold(diff, 20, 255, cv.THRESH_BINARY)
                img_prec_bw = imgBW
                sum = cv.integral(diff) # calcul la somme cumulée de touts les pixel dans l'image
                move = sum[-1][-1]/(diff.shape[0]*diff.shape[1])
                # t1 = time.time()
                # print(t1 - t0)

                # print(move)
                if move >= 10 and sem < 3:
                    sem += 1
                elif move < 10 and sem > -1:
                    sem -= 1
                if sem >= 0 :
                    self.Move = True
                else :
                    self.Move = False
                    
                # print(str(eye)+": "+str(time.time()-tp1))
                # print("move ="+ str(move))
                # print ("sem = " + str(sem))
                # print(str(eye) + " : "+ str(self.Move))

                if self.Move is False:

                    res = Canny_Sharpness_function(imgBW)
                    canny.append(res)
                    poses.append(self.pos[eye])
                    # print (eye+"_res = "+str(res))
                    # print (eye+"_res max = "+str(res_max))
                    # print (eye+"_pos = " + str(self.pos[eye]))

                    self.test_response(self.future_zoom_focus)
                    try:
                        self.zoom = self.future_zoom_focus.result().zoom
                    except  Exception as e:
                        pass

                    if self.zoom < 100:
                        self.bruit = 5


                    if self.Init[eye] is True:
                        # if seuil > res_max and res_max != 0:
                        #     if res_max - 5 > 0:
                        #         seuil = res_max - 5
                        #     else:
                        #         seuil = 0.5
                        # elif seuil < res_max-5:
                        #     seuil = res_max-5
                        print (seuil)
                        self.Init[eye] = False
                        First = True
                        Stop = 0
                        pos_min, pos_max = set_poses(self.zoom)
                        self.pos[eye] = pos_min
                        res_max = 0
                        pas = 1
                        if (eye == "left_eye" and self.Init["right_eye"] is False) or (eye == "right_eye" and self.Init["left_eye"] is False):
                            self.send_request_set_focus_2_cam(pos_min, pos_min)  # déplacement des moteurs aux positions de départs
                            time.sleep(1)
                            print(str(eye) + " initialisation faite")
                            # self.test_response(self.future)
                            self.e_init.set()
                            self.e_init.clear()
                        else:
                            print(str(eye) + " attend l'initialisation")
                            self.e_init.wait()
                    elif Stop == 0:
                        # if len(self.Points) == 2:
                        #     roi = img[im][self.Points[0][1]:self.Points[1][1], self.Points[0][0]:self.Points[1][0]]
                        #     imgBW = cv.cvtColor(roi, cv.COLOR_BGR2GRAY )
                        # else :
                        #     imgBW = cv.cvtColor(img[im], cv.COLOR_BGR2GRAY)

                        # cv.imwrite(str(pos[eye])+"_"+str(k)+".png",img[im])
                        
                        # print("stop = 0")
                        if res > res_max:
                            # print("res > res_max")
                            res_max = res
                            p_max = self.pos[eye]
                        if First is True:
                            # print("First = True")
                            j += 1
                            canny = []  # liste de stockage des résultats donnés par canny
                            poses = []
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
                                self.send_request_set_focus_2_cam(self.final_pos["left_eye"] - 30, self.final_pos["right_eye"] - 30)
                                time.sleep(0.5)
                                print(str(eye) + ": retour en arr")
                                self.send_request_set_focus_2_cam(self.final_pos["left_eye"], self.final_pos["right_eye"])
                                time.sleep(0.5)
                                print(str(eye) + ": pos max atteind")
                                self.e_end.set()
                                # self.test_response(self.future)
                                self.pos[eye] = self.final_pos[eye]
                                self.final_pos[eye] = -1
                                self.e_end.clear()
                            else:
                                print(str(eye) + " attend")
                                self.e_end.wait()
                                self.pos[eye] = self.final_pos[eye]
                                self.final_pos[eye] = -1
                                Stop = 1

                        elif res <= seuil:
                            # print("res<seuil")
                            pas = 10
                            self.pos[eye] = move_to(pos_min, pos_max, self.pos[eye], pas)
                        elif res > borne_sup:
                            # print ("borne dépassée")
                            borne_inf = res-self.bruit
                            borne_sup = res+self.bruit
                            # print ("b_min = "+ str(borne_inf)+"b_max = "+str(borne_sup))
                            pas = 1
                            self.pos[eye] = move_to(pos_min, pos_max, self.pos[eye], pas)
                        else:  # borne_inf<= res <=borne_sup:
                            # print ("dans les bornes")
                            if pas == 1:
                                pas = 5
                            # elif pas == 5:
                                # pas = 10
                            self.pos[eye] = move_to(pos_min, pos_max, self.pos[eye], pas)
                    elif Stop == 1:
                        print ("stop = 1")
                        Stop = 2
                        # if res > res_max :
                        res_max = res
                    elif (res_max < 2 and (res < 0.6*res_max or res > 1.5*res_max)) or (res_max >= 2 and (res < 0.8*res_max or res > 2*res_max)):
                        print("Stop = 2 et image flou ")
                        # print ("res = "+str(res)+", res_max = " + str(res_max))
                        # Stop = 0
                        # First = True
                        self.Init['left_eye'] = True
                        self.Init['right_eye'] = True
                        # self.pos[eye] = pos_min
                        # res_max = 0
                        # pas = 1

                    self.send_request_set_focus_zoom(eye, self.zoom, self.pos[eye])

                    if self.pos[eye] == pos_min:
                        time.sleep(1)
                    if pas == 10:
                        time.sleep(0.15)
                    else:
                        time.sleep(0.1)
                    # if self.pos[eye] == pos_min:
                    #     time.sleep(1)
                    # if pas == 10:
                    #     time.sleep(1)
                    # else :
                    #     time.sleep(1)
                    # self.test_response(self.future)
                else:
                    time.sleep(0.04)
            # if eye == "left_eye":
            #     plt.figure(j)
            #     plt.plot(poses,canny)
            #     plt. grid()
            #     plt.title("résultats traitement d'image par canny en fonction du focus")
            #     plt.xlabel("pas du focus de la lentille")
            #     plt.ylabel("resultat de la fonction de contraste")
            #     plt.savefig("src/reachy_focus/datas/canny_"+str(eye)+str(j)+".png")

    def test_response(self, future):
        i = 0
        while(rclpy.ok()):
            # print(i)
            # i+=1
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                # else:
                #     self.get_logger().info(
                #         f'Result {response.zoom}')
                break
            time.sleep(0.001)
            
    def on_press(self, key):  # callback function pynput appelée à l'activation d'une touche
        if str(key) == "Key.up":  # augmentation du zoom de 1 pas
            if self.zoom + 1 <= 600:
                self.zoom += 1
                self.Init['left_eye'] = True
                self.Init['right_eye'] = True
        if str(key) == "Key.down":  # diminution du zoom de 1 pas
            if (self.zoom - 1) > 0:
                self.zoom -= 1
                self.Init['left_eye'] = True
                self.Init['right_eye'] = True
        if str(key) == "'z'":  # modificarion du zoom
            user_input = input('zoom: ')
            err = 1
            while(err == 1):
                try:
                    self.zoom = int(user_input)
                    if 0 <= self.zoom <= 600:
                        err = 0
                        self.Init['left_eye'] = True
                        self.Init['right_eye'] = True
                    else:
                        user_input = input("le zoom doit être comprs entre 0 et 600, recommencez: ")
                except Exception as e:
                    user_input = input("le zoom saisie n'est pas un int, recommencez: ")
        if str(key) == "'c'":  # clear all, ne garde pas en mémoire la dernière ROI selectionnée
            print("clear all")
            self.Init['left_eye'] = True
            self.Init['right_eye'] = True
            if self.Start is False:
                self.Start = True
            self.Points = []
        if str(key) == "'r'":  # restart, relance la recherche en concervant la ROI selectionnée
            print("restart the sequence")
            self.Init['left_eye'] = True
            self.Init['right_eye'] = True
            if self.Start is False:
                self.Start = True
        # if str(key) == "'i'": # choix d'une zone d'interet
        #     print ("selectonnez une zonne d'intérêt ")
        #     Start = False
        #     Init[eye] = True
        if str(key) == "'s'":  # start/stop
            if self.Start is True:
                self.Start = False
                print("Stop")
            else:
                self.Start = True
                print("Start")
        if str(key) == "'b'":
            self.Start = False
            user_input = input('bruit: ')
            err = 1
            while(err == 1):
                try:
                    self.bruit = float(user_input)
                    break
                except Exception as e:
                    user_input = input("le bruit saisie n'est pas un float, recommencez: ")
            self.Init['left_eye'] = True
            self.Init['right_eye'] = True
            self.Start = True

        # if str(key) == "'p'":
        #     cv.imwrite(str(p)+"_"+str(pos[eye])+".png",img[im])
        #     print (str(p)+"_"+str(pos[eye])+".png saved.")
        #     p+=1
        # if str(key) == "'o'":
        #     cv.imwrite(str(o)+"_"+str(pos[eye])+"with_text.png",im_text)
        #     print (str(o)+"_"+str(pos[eye])+"with_text.png saved.")
        #     o+=1


def main(args=None):

    ######Déclaration des objets######
    rclpy.init(args=args)
    # camera_zoom_client = CameraZoomClient()
    # camera_subscriber = CameraSubscriber()
    camera_focus = CameraFocus()
    # zoom = 350
    # for i in range (0,500, 100):
    #     camera_zoom_client.send_request('right_eye',zoom,i)
    #     while rclpy.ok():
    #         rclpy.spin_once(camera_zoom_client)
    #         if camera_zoom_client.future.done():
    #             try:
    #                 response = camera_zoom_client.future.result()
    #             except Exception as e:
    #                 camera_zoom_client.get_logger().info(
    #                     'Service call failed %r' % (e,))
    #             else:
    #                 camera_zoom_client.get_logger().info(
    #                     f'Result {response.success}')
    #             break
    #     time.sleep(0.5)
    # while(rclpy.ok()):
    #    # rclpy.spin_once(camera_focus)
    try:
        rclpy.spin(camera_focus)
    except KeyboardInterrupt:
        # camera_focus.keyboard_listener.stop()
        camera_focus.destroy_node()
        # camera_zoom_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
