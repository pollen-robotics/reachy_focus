import cv2 as cv
from zoom_kurokesu import ZoomController
from numpy.random import randint
import matplotlib.pyplot as plt
import time

def Canny_Sharpness_function(im):
    im = cv.Canny(im, 50, 100)
    sum = cv.integral(im)  # calcul la somme cumulée de touts les pixel dans l'image
    # print("sum canny = " + str(sum[-1][-1]) )
    return sum[-1][-1]/(im.shape[0]*im.shape[1])

zoom_controller = ZoomController(port = '/dev/ttyACM0')
zoom_controller.homing(side='left')
zoom_controller.homing(side='right')

cap = cv.VideoCapture('/dev/video4')

pos_min = 100
pos_max = 400
max = 0
canny_a = []
poses_a = []
canny_r = []
poses_r = []
side = 'left'

zoom_controller._send_custom_command(side,350,pos_min)
time.sleep(1)
for i in range(pos_min,pos_max,1):
    zoom_controller._send_custom_command(side,350,i)
    time.sleep(0.1)
    ret, img = cap.read()
    print(ret)
    cv.imshow('video',img)
    cv.waitKey(1)
    cv.imwrite(side+"_eye_aller"+str(i)+".png",img)
    canny_a.append(Canny_Sharpness_function(img))
    poses_a.append(i)

plt.figure(1)
plt.plot(poses_a,canny_a,)
plt. grid()
plt.title("résultats traitement d'image par canny en fonction du focus")
plt.xlabel("pas du focus de la lentille")
plt.ylabel("resultat de la fonction de contraste")
plt.savefig(side+"_eye_passe_allee.png")

for i in range(pos_max-1,pos_min+1,-1):
    zoom_controller._send_custom_command(side,350,i)
    time.sleep(0.1)
    ret, img = cap.read()
    print(ret)
    canny_r.append(Canny_Sharpness_function(img))
    poses_r.append(i)
    
plt.figure(2)
plt.plot(poses_r,canny_r)
plt. grid()
plt.title("résultats traitement d'image par canny en fonction du focus")
plt.xlabel("pas du focus de la lentille")
plt.ylabel("resultat de la fonction de contraste")
plt.savefig(side+"_eye_passe_retour.png")

plt.figure(3)
plt.plot(poses_r, canny_r, label='passe retour', c='b')
plt.plot(poses_a, canny_a, label='passe allee', c='r')
plt. grid()
plt.title("résultats traitement d'image par canny en fonction du focus")
plt.xlabel("pas du focus de la lentille")
plt.ylabel("resultat de la fonction de contraste")
plt.savefig(side+"_eye_allee_retour.png")