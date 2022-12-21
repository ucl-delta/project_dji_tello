#!/bin/bash

# Arguments
drone_namespace=tello

source ./launch_tools.bash

ros2 daemon stop

# new_session $drone_namespace ROS_DISCOVERY_SERVER "127.0.0.1:11811" 
new_session $drone_namespace 
# new_window 'domain server' "fastdds discovery --server-id 0"

new_window 'tello_interface' "ros2 launch tello_platform tello_platform.launch.py \
    drone_id:=$drone_namespace"

new_window 'state_estimator' "ros2 launch basic_state_estimator basic_state_estimator_launch.py \
    namespace:=$drone_namespace \
    odom_only:=true"

new_window 'controller_manager' "ros2 launch controller_manager controller_manager_launch.py \
    namespace:=$drone_namespace \
    use_bypass:=true \
    config:=config/controller.yaml"

# new_window 'traj_generator' "ros2 launch trajectory_generator trajectory_generator_launch.py  \
#     drone_id:=$drone_namespace"

new_window 'basic_behaviours' "ros2 launch as2_basic_behaviours all_basic_behaviours_launch.py \
    drone_id:=$drone_namespace \
    config_follow_path:=config/follow_path_behaviour.yaml \
    config_takeoff:=config/takeoff_behaviour.yaml \
    config_land:=config/land_behaviour.yaml \
    config_goto:=config/goto_behaviour.yaml "

new_window 'teleop' "ros2 launch keyboard_teleoperation keyboard_teleoperation.launch.py \
    drone_id:=$drone_namespace \
    verbose:=true"

new_window 'viewer' "gnome-terminal -x bash -c '\
    ros2 run alphanumeric_viewer alphanumeric_viewer_node --ros-args -r  __ns:=/$drone_namespace'"




