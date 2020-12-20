'''
*************************
Robotic Pollinator: Mechanics, Planning, Computer Vision and Serial Communication
*************************
Authors: Nicolás Contreras
Email: nucontreras@uc.cl
Date: December 2020
*************************
'''

import cv2
import numpy as np
import imutils
from absl.flags import FLAGS
import os

Datos = 'kiwi_photos'
if not os.path.exists(Datos):
  print(f'Carpeta creada: {Datos}')
  os.makedirs(Datos)

# cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap = cv2.VideoCapture(0)

x1, y1 = 190, 80
x2, y2 = 450, 398

count = 0

while True:
    ret, frame = cap.read()
    if ret == False:
      print("chaaaa")
      break
    imAux = frame.copy()
    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

    objeto = imAux[y1:y2, x1:x2]
    objeto = imutils.resize(objeto, width=38)

    k = cv2.waitKey(1)
    if k == 27:
      break

    if k == ord('s'):
      cv2.imwrite(Datos + '/objeto_{}.jpg'.format(count), objeto)
      print('Imagen almacenada: ', 'objeto_{}.jpg'.format(count))
      count += 1

    cv2.imshow('frame', frame)
    cv2.imshow('objeto', objeto)
