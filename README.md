# Hiwonder-TurboPi-
An autonomous TurboPi robot that follows curved lanes, avoids obstacles, and responds to traffic light colors : stopping on red, slowing on yellow, and going on green.

Features
Lane following: stays within lane boundaries, including curves
Obstacle avoidance: detects and avoids obstacles in real time
Traffic light detection: recognizes light colors via camera
Red = stop
Yellow = slow down
Green = go
Built With
Hardware: TurboPi robot kit (Raspberry Pi-based)
Software: Python, OpenCV
How It Works
Captures live video from the onboard camera
Detects lane edges and adjusts steering to stay centered, even on curves
Detects obstacles in the robot's path and triggers avoidance maneuvers
Identifies traffic light colors using color detection and adjusts speed accordingly
Getting Started

Prerequisites:

TurboPi robot kit, fully assembled
Python 3.x
OpenCV installed (pip install opencv-python)

Running the robot:

Clone this repo onto your TurboPi's Raspberry Pi
Install dependencies from requirements.txt
Run the main script (python main.py)
Project Structure
src — main code (main.py, lane_detection.py, obstacle_avoidance.py, traffic_light.py)
docs/images — photos and GIFs
requirements.txt
README.md
Known Limitations / Future Work
Lane detection can struggle in low-light conditions
Traffic light detection assumes consistent lighting/color calibration
Planned: smoother speed transitions when slowing for yellow lights
Notes

This project was built as a learning/portfolio project. Shared for demonstration purposes — please do not copy or redistribute without permission.
