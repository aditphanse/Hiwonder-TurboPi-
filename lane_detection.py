import sys
sys.path.append('/home/pi/TurboPi/')

import cv2
import time
import signal
import threading
import numpy as np

import Camera
import HiwonderSDK.Board as Board
import HiwonderSDK.mecanum as mecanum


# INIT


car = mecanum.MecanumChassis()
size = (640, 480)

__isRunning = False

lane_center_x = -1


smoothed_x = 320
alpha = 0.12   


last_error = 0
raw_last_error = 0   


# LANE MEMORY

last_left = None
last_right = None
lane_width_px = 260          
missing_frames = 0
MAX_MISSING_BOTH = 15        





RECOVERY_BIAS_PX = 25


# STOP CAR


def car_stop():
    car.set_velocity(0, 90, 0)


# INIT / START / STOP


def init():
    print("Lane Follow Init")
    car_stop()

def start():
    global __isRunning, lane_center_x, last_error, raw_last_error
    __isRunning = True
    lane_center_x = -1
    last_error = 0
    raw_last_error = 0
    print("Lane Follow Start")

def stop():
    global __isRunning
    __isRunning = False
    car_stop()
    print("Lane Follow Stop")

def exit():
    global __isRunning
    __isRunning = False
    car_stop()
    print("Lane Follow Exit")


# LANE DETECTION


def run(img):
    global lane_center_x, smoothed_x, last_left, last_right
    global lane_width_px, missing_frames

    frame = cv2.resize(img, size)

    roi_y = 230
    roi = frame[roi_y:480, :]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    _, thresh = cv2.threshold(blur, 70, 255, cv2.THRESH_BINARY_INV)
    kernel = np.ones((3, 3), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    height, width = thresh.shape

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for c in contours:
        if cv2.contourArea(c) < 1200:
            continue
        x, y, w, h = cv2.boundingRect(c)
        candidates.append(x + w // 2)

    left_x = None
    right_x = None

    if len(candidates) >= 2:
        candidates.sort()
        left_x = candidates[0]
        right_x = candidates[-1]

    elif len(candidates) == 1:
        cx = candidates[0]
        if lane_center_x != -1:
            if cx < lane_center_x:
                left_x = cx
            else:
                right_x = cx
        elif last_left is not None and last_right is not None:
            if abs(cx - last_left) < abs(cx - last_right):
                left_x = cx
            else:
                right_x = cx
        elif last_left is not None:
            left_x = cx
        elif last_right is not None:
            right_x = cx
        else:
            if cx < width // 2:
                left_x = cx
            else:
                right_x = cx

    if left_x is not None:
        last_left = left_x
    if right_x is not None:
        last_right = right_x

    if left_x is not None and right_x is not None:
        measured_width = right_x - left_x
        if 100 < measured_width < 400:
            lane_width_px = 0.9 * lane_width_px + 0.1 * measured_width

    
    if left_x is None and right_x is not None:
        left_x = right_x - lane_width_px - RECOVERY_BIAS_PX
    elif right_x is None and left_x is not None:
        right_x = left_x + lane_width_px + RECOVERY_BIAS_PX

    if left_x is not None and right_x is not None:
        missing_frames = 0

        center = (left_x + right_x) / 2
        smoothed_x = 0.85 * smoothed_x + 0.15 * center
        lane_center_x = int(smoothed_x)

        cv2.circle(frame, (int(left_x), roi_y + 100), 6, (255, 0, 0), -1)
        cv2.circle(frame, (int(right_x), roi_y + 100), 6, (0, 0, 255), -1)
        cv2.circle(frame, (lane_center_x, roi_y + 100), 6, (0, 255, 0), -1)
        cv2.circle(frame, (320, roi_y + 100), 4, (0, 255, 255), -1)
    else:
        missing_frames += 1
        if missing_frames > MAX_MISSING_BOTH:
            lane_center_x = -1

    return frame, gray, thresh


# MOVEMENT THREAD


Kp = 0.0012
Kd = 0.0004
max_turn = 0.16
DEADZONE = 45
MAX_TURN_STEP = 0.015

smoothed_derivative = 0
last_turn = 0

def move():
    global lane_center_x, __isRunning, last_error, raw_last_error
    global smoothed_derivative, last_turn

    while True:

        if __isRunning:

            if lane_center_x != -1:

                image_center = 320
                raw_error = lane_center_x - image_center

                if abs(raw_error) < DEADZONE:
                    error = 0
                else:
                    error = raw_error - DEADZONE * (1 if raw_error > 0 else -1)

                raw_derivative = raw_error - raw_last_error
                raw_last_error = raw_error
                smoothed_derivative = 0.7 * smoothed_derivative + 0.3 * raw_derivative

                last_error = error

                turn = -(Kp * error + Kd * smoothed_derivative)
                turn *= 0.7
                turn = max(-max_turn, min(max_turn, turn))

                delta = turn - last_turn
                if delta > MAX_TURN_STEP:
                    turn = last_turn + MAX_TURN_STEP
                elif delta < -MAX_TURN_STEP:
                    turn = last_turn - MAX_TURN_STEP
                last_turn = turn

                speed = int(38 - abs(turn) * 40)
                speed = max(20, speed)

                if error == 0:
                    car.set_velocity(speed, 90, 0)
                else:
                    car.set_velocity(speed, 90, turn)

            else:
                car_stop()
                last_turn = 0

        else:
            car_stop()
            last_turn = 0

        time.sleep(0.03)


# THREAD START


th = threading.Thread(target=move)
th.setDaemon(True)
th.start()


# MAIN


if __name__ == '__main__':

    init()
    start()

    camera = Camera.Camera()
    camera.camera_open(correction=True)

    signal.signal(signal.SIGINT, lambda s, f: stop())

    while True:

        img = camera.frame

        if img is not None:

            frame, gray, thresh = run(img)

            cv2.imshow("RGB", frame)
            cv2.imshow("GRAY", gray)
            cv2.imshow("THRESH", thresh)

            if cv2.waitKey(1) == 27:
                break

        else:
            time.sleep(0.01)

    camera.camera_close()
    cv2.destroyAllWindows()
    stop()
