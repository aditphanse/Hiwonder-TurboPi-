import sys
sys.path.append('/home/pi/TurboPi/')

import cv2
import time
import signal
import threading
import numpy as np

import Camera
import yaml_handle
import HiwonderSDK.Board as Board
import HiwonderSDK.mecanum as mecanum
import HiwonderSDK.Sonar as Sonar   


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
RECOVERY_BIAS_PX = 35
FAR_ROI_TOP = 150
FAR_ROI_BOTTOM = 230          
FAR_MIN_CONTOUR_AREA = 400    

curvature_signal = 0          
CURVATURE_SMOOTH = 0.6        
CURVATURE_DECAY = 0.85        


# OBSTACLE AVOIDANCE (ultrasonic)

sonar = Sonar.Sonar()

obstacle_distance_mm = 9999      
STOP_DISTANCE_MM = 200           
SLOW_DISTANCE_MM = 400           
SONAR_POLL_INTERVAL = 0.1        



STOP_CONFIRM_COUNT = 3
_close_reading_streak = 0

_sonar_history = []
SONAR_HISTORY_LEN = 5
was_obstacle_blocked = False


# TRAFFIC LIGHT COLOR CONTROL


lab_data = None
target_color = None   

YELLOW_SPEED_CAP = 27
was_red_blocked = False


def load_color_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)


def get_area_max_contour(contours):
    max_area = 0
    max_contour = None

    for c in contours:
        area = abs(cv2.contourArea(c))
        if area > max_area and area > 500:
            max_area = area
            max_contour = c

    return max_contour, max_area


def detect_color(img):
    global lab_data

    frame = cv2.resize(img, size)
    frame_blur = cv2.GaussianBlur(frame, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2LAB)

    detected = None
    max_area = 0

    for color in ["red", "yellow", "green"]:

        mask = cv2.inRange(
            frame_lab,
            tuple(lab_data[color]['min']),
            tuple(lab_data[color]['max'])
        )

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        c, area = get_area_max_contour(contours)

        if c is not None and area > max_area:
            max_area = area
            detected = color

    return detected


# STOP CAR


def car_stop():
    car.set_velocity(0, 90, 0)


# INIT / START / STOP


def init():
    print("Lane Follow + Traffic Light Init")
    load_color_config()
    car_stop()

def start():
    global __isRunning, lane_center_x, last_error, raw_last_error
    __isRunning = True
    lane_center_x = -1
    last_error = 0
    raw_last_error = 0
    print("Lane Follow + Traffic Light Start")

def stop():
    global __isRunning
    __isRunning = False
    car_stop()
    print("Lane Follow + Traffic Light Stop")

def exit():
    global __isRunning
    __isRunning = False
    car_stop()
    print("Lane Follow + Traffic Light Exit")


# SONAR POLLING THREAD (unchanged from lane-assist script)


def sonar_loop():
    global obstacle_distance_mm, _sonar_history
    while True:
        try:
            d = sonar.getDistance()
            if d is not None and d > 0:
                _sonar_history.append(d)
                if len(_sonar_history) > SONAR_HISTORY_LEN:
                    _sonar_history.pop(0)
                sorted_hist = sorted(_sonar_history)
                obstacle_distance_mm = sorted_hist[len(sorted_hist) // 2]
        except Exception as e:
            print("Sonar read error:", e)
        time.sleep(SONAR_POLL_INTERVAL)

sonar_thread = threading.Thread(target=sonar_loop)
sonar_thread.setDaemon(True)
sonar_thread.start()


# LANE DETECTION (unchanged - do not touch)


def run(img):
    global lane_center_x, smoothed_x, last_left, last_right
    global lane_width_px, missing_frames
    global curvature_signal

    frame = cv2.resize(img, size)

    
    far_roi = frame[FAR_ROI_TOP:FAR_ROI_BOTTOM, :]
    far_gray = cv2.cvtColor(far_roi, cv2.COLOR_BGR2GRAY)
    far_blur = cv2.GaussianBlur(far_gray, (5, 5), 0)
    _, far_thresh = cv2.threshold(far_blur, 70, 255, cv2.THRESH_BINARY_INV)
    far_thresh = cv2.morphologyEx(far_thresh, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

    far_contours, _ = cv2.findContours(far_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    far_candidates = []
    for c in far_contours:
        if cv2.contourArea(c) < FAR_MIN_CONTOUR_AREA:
            continue
        x, y, w, h = cv2.boundingRect(c)
        far_candidates.append(x + w // 2)

    if len(far_candidates) >= 2:
        far_candidates.sort()
        far_center = (far_candidates[0] + far_candidates[-1]) / 2
        raw_curvature = far_center - 320
        curvature_signal = CURVATURE_SMOOTH * curvature_signal + (1 - CURVATURE_SMOOTH) * raw_curvature
        cv2.circle(frame, (int(far_center), FAR_ROI_TOP + 40), 5, (255, 0, 255), -1)
    else:
        
        
        curvature_signal *= CURVATURE_DECAY

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


Kp = 0.0009
Kd = 0.0004
Kp2 = 0.0000015
Kc = 0.00035
max_turn = 0.20          
DEADZONE = 45
MAX_TURN_STEP = 0.02      
MAX_SPEED_STEP = 3
last_speed = 0
smoothed_derivative = 0
last_turn = 0

def move():
    global lane_center_x, __isRunning, last_error, raw_last_error
    global smoothed_derivative, last_turn
    global was_obstacle_blocked, last_left, last_right, missing_frames
    global _close_reading_streak
    global was_red_blocked, target_color
    global last_speed, curvature_signal

    while True:

        if __isRunning:

            
            # OBSTACLE CHECK (unchanged) - highest priority, hard e-stop
            
            obstacle_speed_cap = None

            if obstacle_distance_mm <= STOP_DISTANCE_MM:
                _close_reading_streak += 1
            else:
                _close_reading_streak = 0

            is_blocked_now = _close_reading_streak >= STOP_CONFIRM_COUNT

            if is_blocked_now:
                car_stop()
                last_turn = 0
                last_speed = 0
                was_obstacle_blocked = True
                time.sleep(0.03)
                continue

            if was_obstacle_blocked and not is_blocked_now:
                last_left = None
                last_right = None
                missing_frames = 0
                raw_last_error = 0
                smoothed_derivative = 0
                last_turn = 0
                was_obstacle_blocked = False
           
            is_red_now = (target_color == "red")

            if is_red_now:
                car_stop()
                last_turn = 0
                last_speed = 0
                was_red_blocked = True
                time.sleep(0.03)
                continue

            if was_red_blocked and not is_red_now:
                last_left = None
                last_right = None
                missing_frames = 0
                raw_last_error = 0
                smoothed_derivative = 0
                last_turn = 0
                was_red_blocked = False
            
            if obstacle_distance_mm < SLOW_DISTANCE_MM:
                span = SLOW_DISTANCE_MM - STOP_DISTANCE_MM
                frac = (obstacle_distance_mm - STOP_DISTANCE_MM) / span
                obstacle_speed_cap = int(20 + frac * 18)
            
            if target_color == "yellow":
                if obstacle_speed_cap is None:
                    obstacle_speed_cap = YELLOW_SPEED_CAP
                else:
                    obstacle_speed_cap = min(obstacle_speed_cap, YELLOW_SPEED_CAP)

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
                proportional = Kp * error + Kp2 * error * abs(error)
                turn = -(proportional + Kd * smoothed_derivative + Kc * curvature_signal)
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

                if obstacle_speed_cap is not None:
                    speed = min(speed, obstacle_speed_cap)
                speed_delta = speed - last_speed
                if speed_delta > MAX_SPEED_STEP:
                    speed = last_speed + MAX_SPEED_STEP
                elif speed_delta < -MAX_SPEED_STEP:
                    speed = last_speed - MAX_SPEED_STEP
                last_speed = speed

                if error == 0:
                    car.set_velocity(speed, 90, 0)
                else:
                    car.set_velocity(speed, 90, turn)

            else:
                car_stop()
                last_turn = 0
                last_speed = 0

        else:
            car_stop()
            last_turn = 0
            last_speed = 0

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

            target_color = detect_color(img)

            frame, gray, thresh = run(img)

            cv2.putText(frame, f"Light: {target_color}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(frame, f"Obstacle: {obstacle_distance_mm}mm", (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

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
