'''
*************************
Robotic Pollinator: Mechanics, Planning, Computer Vision and Serial Communication
*************************
Authors: Daniel Medina, Nicolás Contreras
Email: dlmedina@uc.cl, nucontreras@uc.cl
Date: December 2020
*************************
'''

from serial_communication import init_serial, close_serial, serial_com, se_data
from frames_c import Frames
import threading
import numpy as np
import time

from absl import app, flags, logging
from absl.flags import FLAGS
import YOLOKiwiFlower.computer_vision as cv


flags.DEFINE_string('classes', './YOLOKiwiFlower/data/labels/obj.names', 'path to classes file')
flags.DEFINE_string('weights', './YOLOKiwiFlower/weights/yolov3.tf',
                    'path to weights file')
flags.DEFINE_integer('size', 416, 'resize images to')
flags.DEFINE_string('video', './YOLOKiwiFlower/data/video/paris.mp4',
                    'path to video file or number for webcam)') 
flags.DEFINE_integer('num_classes', 1, 'number of classes in the model')


if __name__ == '__main__':
    try:
        THETA_ERR = 0.01 # radb
    
        # serial comm routines
        ser = init_serial()
        thread_ser = threading.Thread(target=serial_com, args=(ser,))
        thread_ser.start()

        vc_thread = threading.Thread(target = cv.vc_robotic_arm)
        vc_thread.start()

        # reference frames
        frames = Frames()
        
        input()
        cv.coordinates.search_positions = True

        flowers_pollinated = False
        while not flowers_pollinated:
            time.sleep(0.2)    

            if cv.coordinates.coordinates_found:
                print("The coordinates are:")
                print(cv.coordinates.positions)
                angles = input('angles: ')
                angles = angles.strip().split(';')
                th1, th2, th3, th4  = [float(theta) for theta in angles]
                # path planning
                positions = cv.coordinates.positions.copy()
                flower_index = 0
                print("RUNNING PATH PLANNING\n")                
                for pos in positions:
                    thetas = [th1, th2, th3, th4]
                    frames.AddTrans(*thetas, *pos)

                    sol, founded = frames.IKinEE2(*thetas, *pos, choose=True, draw=True)
                    if founded:
                        se_data.theta1Goal = sol[0]
                        se_data.theta2Goal = sol[1]
                        se_data.theta3Goal = sol[2]
                        se_data.theta4Goal = sol[3]

                        print(f'flower {flower_index} config founded! :)')
                        msg_str = f'{se_data.theta1Goal:.3f};{se_data.theta2Goal:.3f};' + \
                                f'{se_data.theta3Goal:.3f};{se_data.theta4Goal:.3f};{se_data.servo}'
                        print(msg_str)            

                        frames.DrawFrames(sol=True)

                        while (se_data.theta1Goal - se_data.theta1 > THETA_ERR) and \
                              (se_data.theta2Goal - se_data.theta2 > THETA_ERR) and \
                              (se_data.theta3Goal - se_data.theta3 > THETA_ERR) and \
                              (se_data.theta4Goal - se_data.theta4 > THETA_ERR):
                            print(f'reaching flower {flower_index}...')
                            time.sleep(0.050)
                        
                        print(f'flower {flower_index} reached!!')

                        # pollinate
                        se_data.servo = 'on'
                        time.sleep(0.200)
                        se_data.servo = 'off'
                        time.sleep(0.200)
                        print(f'flower {flower_index} pollinated!!')

                    else:
                        print(f'flower {flower_index} config NOT founded :(')
                    
                    flower_index += 1

                print('ALL FLOWERS WERE POLLINATED\n')

                # return to home
                se_data.theta1Goal = 0
                se_data.theta2Goal = np.pi/2 - frames.alpha
                se_data.theta3Goal = -np.pi + frames.beta
                se_data.theta4Goal = -frames.gamma

                flowers_pollinated = True

    except SystemExit:
        pass