# RMIT robot firmware repo

This is a clone of rmitbot_firmware located in the `lesson6_ws` repository of `ROS-Autobot`, at `src/rmitbot_firmware/arduino_firmware/rmitbot_arduino_firmware`

It will be ran on an ESP32 located on a robot running ROS2 on a Raspberry Pi 5. It will mainly be used to drive four motors and their motor drivers alongside filtering the data from the IMU to minimize its natural drift.

## Changes

- expansion to four (4) motors and their IBT_2 motor controllers
- some IMU support for sending relevant information to the Raspberry Pi located on the robot