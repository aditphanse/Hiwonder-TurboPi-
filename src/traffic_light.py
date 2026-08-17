import sys
sys.path.append('/home/pi/TurboPi/')

import cv2
import time
import signal
import threading
import numpy as np

import Camera
import yaml_handle
import HiwonderSDK.Sonar as Sonar
import HiwonderSDK.Board as Board
import HiwonderSDK.mecanum as mecanum


# INIT


car = mecanum.MecanumChassis()
HWSONAR = Sonar.Sonar()

size = (640, 480)

__isRunning = False

distance = 999

target_color = None

lab_data = None


# LOAD COLOR CONFIG


def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)


# STOP CAR


def car_stop():
    car.set_velocity(0, 90, 0)


# INIT


def init():
    load_config()
    car_stop()
    print("Color Traffic Control Init")

def start():
    global __isRunning
    __isRunning = True
    print("Started")

def stop():
    global __isRunning
    __isRunning = False
    car_stop()
    print("Stopped")

def exit():
    global __isRunning
    __isRunning = False
    car_stop()
    print("Exit")


# GET LARGEST COLOR OBJECT


def get_area_max_contour(contours):
    max_area = 0
    max_contour = None

    for c in contours:
        area = abs(cv2.contourArea(c))
        if area > max_area and area > 500:
            max_area = area
            max_contour = c

    return max_contour, max_area


# COLOR DETECTION


def detect_color(img):
    global lab_data

    frame = cv2.resize(img, size)
    frame_blur = cv2.GaussianBlur(frame, (3,3), 3)
    frame_lab = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2LAB)

    detected = None
    max_area = 0

    for color in ["red", "yellow", "green"]:

        mask = cv2.inRange(
            frame_lab,
            tuple(lab_data[color]['min']),
            tuple(lab_data[color]['max'])
        )

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        c, area = get_area_max_contour(contours)

        if c is not None and area > max_area:
            max_area = area
            detected = color

    return detected


# SENSOR THREAD


def sensor_loop():
    global distance

    while True:

        dist = HWSONAR.getDistance() / 10.0

        if dist <= 0:
            dist = 999

        distance = dist

        time.sleep(0.05)


# MOVE LOGIC


def move():
    global distance, target_color, __isRunning

    while True:

        if __isRunning:

            
            
            
            if distance < 25:
                car_stop()
                time.sleep(0.05)
                continue

            
            # COLOR CONTROL
            
            if target_color == "red":
                car_stop()

            elif target_color == "yellow":
                car.set_velocity(30, 90, 0)

            elif target_color == "green":
                car.set_velocity(45, 90, 0)

            else:
                car.set_velocity(20, 90, 0)

        else:
            car_stop()

        time.sleep(0.05)


# THREADS


threading.Thread(target=sensor_loop, daemon=True).start()
threading.Thread(target=move, daemon=True).start()


# MAIN LOOP


if __name__ == '__main__':

    init()
    start()

    camera = Camera.Camera()
    camera.camera_open(correction=True)

    signal.signal(signal.SIGINT, lambda s,f: stop())

    while True:

        img = camera.frame

        if img is not None:

            target_color = detect_color(img)

            
            cv2.putText(img, f"Color: {target_color}", (30, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)

            cv2.putText(img, f"Dist: {distance:.1f}cm", (30, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,0), 2)

            cv2.imshow("TurboPi Control", img)

            if cv2.waitKey(1) == 27:
                break

        else:
            time.sleep(0.01)

    camera.camera_close()
    cv2.destroyAllWindows()
    stop()
