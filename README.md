# Hiwonder-TurboPi-
An autonomous TurboPi robot that follows curved lanes, avoids obstacles, and responds to traffic light colors — stopping on red, slowing on yellow, and going on green.

<!-- Add a photo or GIF here once you have one --> <!-- ![TurboPi demo](docs/images/demo.gif) -->
Features
🛣️ Lane following — stays within lane boundaries, including curves
🚧 Obstacle avoidance — detects and avoids obstacles in real time
🚦 Traffic light detection — recognizes light colors via camera:
🔴 Red → stop
🟡 Yellow → slow down
🟢 Green → go
Built With
Hardware: TurboPi robot kit (Raspberry Pi-based)
Software: Python, OpenCV
How It Works

The robot uses its onboard camera to capture live video frames, which are processed with OpenCV to:

Detect lane edges/markings and calculate steering adjustments to stay centered, even on curves
Detect obstacles in the robot's path and trigger avoidance maneuvers
Identify traffic light colors (red/yellow/green) using color detection and adjust speed accordingly
<!-- Feel free to expand this section with more detail on your algorithm, e.g. what color space you use, how you detect lane edges, how steering angle is calculated, how obstacle distance is measured, etc. -->
Getting Started
Prerequisites
TurboPi robot kit, fully assembled
Python 3.x
OpenCV installed:
  pip install opencv-python
Running the Robot
Clone this repo onto your TurboPi's Raspberry Pi:
   git clone https://github.com/your-username/turbopi-robot.git
Install dependencies:
   pip install -r requirements.txt
Run the main script:
   python main.py
Project Structure
turbopi-robot/
├── src/                  # main code
│   ├── main.py           # entry point
│   ├── lane_detection.py
│   ├── obstacle_avoidance.py
│   └── traffic_light.py
├── docs/
│   └── images/           # photos/GIFs
├── requirements.txt
└── README.md
Known Limitations / Future Work
<!-- Great for a portfolio piece — shows self-awareness. Fill in real ones, e.g.: -->
Lane detection can struggle in low-light conditions
Traffic light detection assumes consistent lighting/color calibration
Planned: smoother speed transitions when slowing for yellow lights
Notes

This project was built as a learning/portfolio project. Shared for demonstration purposes — please do not copy or redistribute without permission.
