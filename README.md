# Drone code
## Setup
- This is intended to run on a Clover COEX drone.
- You will need to adjust the camera launch file so the camera topic is properly sized (width of 640, height of 480).
- You will need to adjust the rangefinder setup.

## Usage
- Make sure you're connected to the `jetnet` network.
- To run the rangefinder:
  - `source ~/catkin_ws/devel/setup.bash`
  - `roslaunch vl53l0x drone.launch`
  - `rosservice call /vl53l0x/start_ranging 2`
- To run the image-processing node:
  - `source ~/catkin_ws/devel/setup.bash`
  - `rosrun image_handler image_handler`

## Common Issues
### RPi Networking
- If you run into issues with connecting to the Jetson's hotspot:
  - Try poking at it with `wpa_cli`.
  - If you see something like `wlan0: CTRL-EVENT-ASSOC-REJECT bssid=**:**:**:**:**:** status_code=16`,
  - try the solution described in [this answer](https://raspberrypi.stackexchange.com/a/140524).

## Tasks
- ~Turn this into a proper ROS node and workspace~
- ~Test the image script~
- Make the socket handling more robust
- More detail on rangefinder setup.
