#!/bin/bash

DRONE_ID_NAMESPACE1="drone_sim_0"
DRONE_ID_NAMESPACE2="drone_sim_1"

mkdir rosbags 
cd rosbags &&\
ros2 bag record \
"/clock" \
"/tello/actuator_command/pose" \
"/tello/actuator_command/thrust" \
"/tello/actuator_command/twist" \
"/tello/controller/info" \
"/tello/motion_reference/pose" \
"/tello/motion_reference/trajectory" \
"/tello/motion_reference/twist" \
"/tello/platform/info" \
"/tello/self_localization/pose" \
"/tello/self_localization/twist" \
"/tello/sensor_measurements/barometer" \
"/tello/sensor_measurements/battery" \
"/tello/sensor_measurements/imu" \
"/tello/sensor_measurements/odom" \
"/tf" \
"/tf_static" \
--qos-profile-overrides-path $(pwd)/../reliability_override.yaml \
--include-hidden-topics