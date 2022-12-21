#!/bin/bash

# Arguments
drone_namespace=tello

source ./launch_tools.bash

ros2 daemon stop

# new_session $drone_namespace ROS_DISCOVERY_SERVER "127.0.0.1:11811" 
new_session $drone_namespace 
# new_window 'domain server' "fastdds discovery --server-id 0"

new_window 'tello_interface' "ros2 launch as2_tello_platform tello_platform.launch.py \
    drone_id:=$drone_namespace"

new_window 'state_estimator' "ros2 launch as2_state_estimator state_estimator_launch.py \
    namespace:=$drone_namespace \
    plugin_name:=as2_state_estimator_plugin_external_odom"

new_window 'as2_controller_manager' "ros2 launch as2_controller_manager controller_manager_launch.py \
    namespace:=$drone_namespace \
    cmd_freq:=100.0 \
    info_freq:=10.0 \
    use_bypass:=true \
    plugin_name:=controller_plugin_speed_controller \
    plugin_config_file:=config/default_controller.yaml"


# new_window 'traj_generator' "ros2 launch trajectory_generator trajectory_generator_launch.py  \
#     drone_id:=$drone_namespace"

new_window 'as2_platform_behaviors' "ros2 launch as2_platform_behaviors as2_platform_behaviors_launch.py \
    namespace:=$drone_namespace \
    follow_path_plugin_name:=follow_path_plugin_position \
    goto_plugin_name:=goto_plugin_position \
    takeoff_plugin_name:=takeoff_plugin_platform \
    land_plugin_name:=land_plugin_platform "

new_window 'teleop' "ros2 launch as2_keyboard_teleoperation as2_keyboard_teleoperation_launch.py \
    drone_id:=$drone_namespace \
    verbose:=true"

new_window 'viewer' "gnome-terminal -x bash -c '\
    ros2 run as2_alphanumeric_viewer as2_alphanumeric_viewer_node --ros-args -r  __ns:=/$drone_namespace'"


tmux attach-session -t $drone_namespace

