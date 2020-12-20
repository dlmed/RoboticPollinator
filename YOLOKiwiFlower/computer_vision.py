'''
*************************
Robotic Pollinator: Mechanics, Planning, Computer Vision and Serial Communication
*************************
Authors: Nicolás Contreras
Email: nucontreras@uc.cl
Date: December 2020
*************************
'''

import time
import numpy as np
from absl import app, flags, logging
from absl.flags import FLAGS
import cv2
import tensorflow as tf
from YOLOKiwiFlower.yolov3_tf2.models import (
    YoloV3, YoloV3Tiny
)
from YOLOKiwiFlower.yolov3_tf2.dataset import transform_images
from YOLOKiwiFlower.yolov3_tf2.utils import draw_outputs
from threading import Thread
from time import sleep


class Positions:
    def __init__(self):
        self.positions = list()
        self.coordinates_found = False
        self.search_positions = False

coordinates = Positions()

def main(_argv):
    physical_devices = tf.config.experimental.list_physical_devices('GPU')
    if len(physical_devices) > 0:
        tf.config.experimental.set_memory_growth(physical_devices[0], True)
    yolo = YoloV3(classes=FLAGS.num_classes)
    yolo.load_weights(FLAGS.weights)
    logging.info('weights loaded')
    class_names = [c.strip() for c in open(FLAGS.classes).readlines()]
    logging.info('classes loaded')
    try:
        vid = cv2.VideoCapture(int(FLAGS.video))
    except:
        vid = cv2.VideoCapture(FLAGS.video)
    fps = 0.0
    count = 0
    # positions = list()
    search_continue = True
    while True:
        _, img = vid.read()

        if img is None:
            logging.warning("Empty Frame")
            time.sleep(0.1)
            count += 1
            if count < 3:
                continue
            else:
                break

        # Resize windows
        img_in = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_in = tf.expand_dims(img_in, 0)
        img_in = transform_images(img_in, FLAGS.size)
        
        # Image size (center screen)
        height, width, channels = img.shape  # Esto se puede borrar y dejar la medida fija
        center = (int(width / 2), int(height / 2))
        cv2.circle(img, (center), 3, (255, 255, 0), -1)

        if search_continue:
            # t1 = time.time()
            boxes, scores, classes, nums = yolo.predict(img_in)
            # fps = (fps + (1. / (time.time() - t1))) / 2
            # img = cv2.putText(img, "FPS: {:.2f}".format(fps), (0, 30),
            #                   cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 255), 2)  # Muestra los FPS.

            img, coordinates.positions = draw_outputs(img, (boxes, scores, classes, nums), class_names, img, coordinates.positions, center)

            if coordinates.search_positions and len(coordinates.positions) != 0 and len(coordinates.positions[0][0]) >= 10: 
                ready = True
                for coord in coordinates.positions:
                    sd_x = np.std(coord[0])
                    sd_y = np.std(coord[1])
                    sd_z = np.std(coord[2])
                    coord[0] = round(np.mean(coord[0]), 3)
                    coord[1] = round(np.mean(coord[1]), 3)
                    coord[2] = round(np.mean(coord[2]), 3)
                    # print(f"sd_x es {sd_x}, sd_y es {sd_y} y sd_z es {sd_z}")
                    # print(f"la coordenada es {coord}")
                    if sd_x >= 0.8 or sd_y >= 0.8 or sd_z >= 2.8:
                        coordinates.positions = list()
                        ready = False
                        break
                if ready:
                    search_continue = False
                    coordinates.coordinates_found = True

        cv2.imshow('Computer Vision', img)
        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()

def vc_robotic_arm():
    app.run(main)

def thread_prueba():
    while True:
        print("Hola")
        sleep(4)
