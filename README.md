# RMIT robot firmware repo

This is a clone of rmitbot_firmware located in the `lesson6_ws` repository of `ROS-Autobot`, at `src/rmitbot_firmware/arduino_firmware/rmitbot_arduino_firmware`. The link to the repository is `https://github.com/ROS2-AutoBot/lesson6_ws`.

It will be ran on an ESP32 located on a robot running ROS2 on a Raspberry Pi 5. It will mainly be used to drive four motors and their motor drivers alongside filtering the data from the IMU using Kalman filtering to minimize its natural drift. All information will be communicated with the Raspberry Pi 5 via serial through a USB-C cable.

## Changes

- expansion to four (4) motors and their IBT_2 motor controllers
- some IMU support for sending relevant information to the Raspberry Pi located on the robot
