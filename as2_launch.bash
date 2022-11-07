#!/bin/bash

# Arguments
drone_namespace=tello

source ./launch_tools.bash

new_session $drone_namespace

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
    config_goto:=config/goto_behaviour.yaml \
    use_sim_time:=true"

new_window 'rqt_img_view' "ros2 run rqt_image_view rqt_image_view"
