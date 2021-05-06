from __future__ import print_function
from ipywidgets import interact, interactive, fixed, interact_manual
import ipywidgets as widgets
import cv2 as cv
import threading
import matplotlib.pyplot as plt
import numpy as np
import time
import random
from zoom_kurokesu import ZoomController

zoom_controller = ZoomController()
zoom_controller.homing(side='left')
zoom_controller.homing(side='right')

zoom = 350
pos_max = 500


cap_left = cv.VideoCapture('/dev/video0') # left image  #right eye
cap_right = cv.VideoCapture('/dev/video4') # right image  #left eye


ret_left, img_left = cap_left.read()
ret_right, img_right = cap_right.read()


class Video (threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.Terminated = False

    def run(self):
        global cap_left, cap_right
        global img_left, ret_left, img_right,ret_right

        while (1):

            ret_left, img_left = cap_left.read()
            ret_right, img_right = cap_right.read()
            #plt.figure(3)
            # cv.imshow('img_left', img_left)
            # cv.imshow('img_right', img_right)
    def stop(self):
        Terminated = True

def Canny_Sharpness_function (im):
    im = cv.Canny(im,50,100)
    #cv.imwrite('canny.png', im)
    sum = cv.integral(im) # calcul la somme cumulée de touts les pixel dans l'image
    return sum[-1][-1]/(im.shape[0]*im.shape[1])
            
pos_min = 200
pos_max = 320

thread = Video()
thread.start()
time.sleep(2)

it = 500
poses = []

for i in range(it):
    poses.append(random.randint(pos_min, pos_max))
with open('poses.npy', 'wb') as f:
    np.save(f, np.array(poses))
with open('poses.npy', 'rb') as f:
    a = np.load(f)
print(a)
    
res_left = []
res_right = []
zoom_controller._send_custom_command('left',zoom,0)
time.sleep(1.4)
zoom_controller._send_custom_command('right',zoom,0)
time.sleep(1.4)
val = 20

for j in range(len(poses)):
    if j >0 and poses[j-1] > poses[j]:
        zoom_controller._send_custom_command('left',zoom,poses[j]-val) #modifie right image 
        time.sleep(1)
        zoom_controller._send_custom_command('right',zoom,poses[j]-val) #modifie left image
        time.sleep(1)

    zoom_controller._send_custom_command('left',zoom,poses[j])
    time.sleep(1)
    zoom_controller._send_custom_command('right',zoom,poses[j])
    time.sleep(1) 
    print (ret_left)
    if ret_left == True:
        print (j)
        imgBW_left = cv.cvtColor(img_left, cv.COLOR_BGR2GRAY)
        res_left.append(Canny_Sharpness_function(imgBW_left))  
        imgBW_right = cv.cvtColor(img_right, cv.COLOR_BGR2GRAY)
        res_right.append(Canny_Sharpness_function(imgBW_right))   

with open('canny_left.npy', 'wb') as f1:
    np.save(f1, np.array(res_left))

with open('canny_right.npy', 'wb') as f2:
    np.save(f2, np.array(res_right))

print (len(poses))
print (len(res_left))
plt.figure(1,figsize=(15, 9))
plt.scatter(poses, res_left, s = 6)
plt.xlabel("Position de focus de la lentille", fontsize = 'xx-large')
plt.ylabel("Resultat de la fonction de contraste",fontsize = 'xx-large')
plt.savefig("canny_left.png")

plt.figure(2,figsize=(15, 9))
plt.scatter(poses, res_right, s = 6)
plt.xlabel("Position de focus de la lentille", fontsize = 'xx-large')
plt.ylabel("Resultat de la fonction de contraste",fontsize = 'xx-large')
plt.savefig("canny_right.png")