# Drone code
- Right now, this is just a Python script that takes a picture and sends it to the Jetson for further processing.

## Common Issues
### RPi Networking
- If you run into issues with connecting to the Jetson's hotspot:
  - Try poking at it with `wpa_cli`.
  - If you see something like `wlan0: CTRL-EVENT-ASSOC-REJECT bssid=**:**:**:**:**:** status_code=16`,
  - try the solution described in [this answer](https://raspberrypi.stackexchange.com/a/140524).

## Tasks
- Turn this into a proper ROS node and workspace
- Test the image script
- Make the socket handling more robust
