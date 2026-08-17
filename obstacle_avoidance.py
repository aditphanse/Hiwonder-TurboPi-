import sys
sys.path.append('/home/pi/TurboPi/')
import cv2
import time
import math
import signal
import Camera
import threading
import numpy as np
import yaml_handle
import pandas as pd
import HiwonderSDK.Sonar as Sonar
import HiwonderSDK.Board as Board
import HiwonderSDK.mecanum as mecanum

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

car = mecanum.MecanumChassis()

speed = 40
old_speed = 0
distance = 500
Threshold = 30.0
distance_data = []

TextSize = 12
TextColor = (0, 255, 255)

turn = True
forward = True
HWSONAR = None
stopMotor = True
__isRunning = False

def initMove():
    car.set_velocity(0,90,0)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)
    servo1 = servo_data['servo1']
    servo2 = servo_data['servo2']
    Board.setPWMServoPulse(1, servo1, 1000)
    Board.setPWMServoPulse(2, servo2, 1000)

def reset():
    global turn
    global speed
    global forward
    global distance
    global old_speed
    global Threshold
    global stopMotor
    global __isRunning
    
    speed = 40
    old_speed = 0
    distance = 500
    Threshold = 30.0
    turn = True
    forward = True
    stopMotor = True
    __isRunning = False
    
def init():
    print("Avoidance Init")
    initMove()
    reset()
    
__isRunning = False

def start():
    global __isRunning
    global stopMotor
    global forward
    global turn
    
    turn = True
    forward = True
    stopMotor = True
    __isRunning = True
    print("Avoidance Start")

def stop():
    global __isRunning
    __isRunning = False
    car.set_velocity(0,90,0)
    time.sleep(0.3)
    car.set_velocity(0,90,0)
    print("Avoidance Stop")

def exit():
    global __isRunning
    __isRunning = False
    car.set_velocity(0,90,0)
    time.sleep(0.3)
    car.set_velocity(0,90,0) 
    HWSONAR.setPixelColor(0, Board.PixelColor(0, 0, 0))
    HWSONAR.setPixelColor(1, Board.PixelColor(0, 0, 0))
    print("Avoidance Exit")

def setSpeed(args):
    global speed
    speed = int(args[0])
    return (True, ())
    
def setThreshold(args):
    global Threshold
    Threshold = args[0]
    return (True, (Threshold,))

def getThreshold(args):
    global Threshold
    return (True, (Threshold,))

def move():
    global turn
    global speed
    global forward
    global distance
    global Threshold
    global old_speed
    global stopMotor
    global __isRunning

    while True:
        if __isRunning:   
            if speed != old_speed:   
                old_speed = speed
                car.set_velocity(speed,90,0) 
                
            if distance <= Threshold:   
                if turn: 
                    turn = False
                    forward = True
                    stopMotor = True
                    car.set_velocity(0,90,-0.5) 
                    time.sleep(0.5)
                
            else:
                if forward: 
                    turn = True
                    forward = False
                    stopMotor = True
                    car.set_velocity(speed,90,0) 
        else:
            if stopMotor:
                stopMotor = False
                car.set_velocity(0,90,0)  
            turn = True
            forward = True
            time.sleep(0.03)
 
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

def run(img):
    global HWSONAR
    global distance
    global distance_data
    
    dist = HWSONAR.getDistance() / 10.0 

    distance_data.append(dist) 
    data = pd.DataFrame(distance_data)
    data_ = data.copy()
    u = data_.mean()  
    std = data_.std()  

    data_c = data[np.abs(data - u) <= std]
    distance = data_c.mean()[0]

    if len(distance_data) == 5: 
        distance_data.remove(distance_data[0])

    return cv2.putText(img, "Dist:%.1fcm"%distance, (30, 480-30), cv2.FONT_HERSHEY_SIMPLEX, 1.2, TextColor, 2) 



def manual_stop(signum, frame):
    global __isRunning
    
    print('关闭中...')
    __isRunning = False
    car.set_velocity(0,90,0)  

if __name__ == '__main__':
    init()
    start()
    HWSONAR = Sonar.Sonar()
    camera = Camera.Camera()
    camera.camera_open(correction=True) 
    signal.signal(signal.SIGINT, manual_stop)
    while __isRunning:
        img = camera.frame
        if img is not None:
            frame = img.copy() 
            Frame = run(frame)  
            frame_resize = cv2.resize(Frame, (320, 240)) 
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    camera.camera_close()
    cv2.destroyAllWindows()
    
