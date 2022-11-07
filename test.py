"""Drone interface test"""
#!/bin/python3

import sys
from time import sleep
import rclpy
from python_interface.drone_interface import DroneInterface


def command_pose_test(uav: DroneInterface):
    """Pose test"""
    uav.send_motion_reference_pose(position=[0.0, 0.0, 0.5])
    sleep(2)

    uav.send_motion_reference_pose(position=[0.0, 0.0, 1.0])
    sleep(2)

    uav.send_motion_reference_pose(position=[0.0, 0.0, 0.3])
    sleep(2)


def command_vel_test(uav: DroneInterface):
    """Vel test"""
    uav.send_motion_reference_twist(
        lineal=[0.0, 0.0, 0.0], angular=[0.0, 0.0, 1.0])
    sleep(2)

    uav.send_motion_reference_twist(
        lineal=[0.0, 0.0, 0.0], angular=[0.0, 0.0, -1.0])
    sleep(2)

    uav.send_motion_reference_twist(
        lineal=[0.0, 0.0, 0.0], angular=[0.0, 0.0, 0.0])
    sleep(2)


def drone_run(drone_interface: DroneInterface):
    """drone run"""

    speed = 4.0
    height = 6.0

    goal0 = [20.0, 0.0, height]
    goal1 = [20.0, 20.0, height]
    goal2 = [0.0, 0.0, height]

    print("Start mission")

    drone_interface.offboard()
    drone_interface.arm()
    print("Take Off")
    drone_interface.takeoff()  # Takeoff platform, cannot choose height or speed
    print("Take Off done")

    sleep(1)
    print("Test motion ref pose")
    command_vel_test(drone_interface)

    # ##### GOTO #####
    # sleep(1.0)
    # print(f"Go to: [{goal0}]")
    # drone_interface.go_to_point(goal0, speed=speed, ignore_yaw=False)
    # print("Go to done")

    # sleep(1.0)
    # print(f"Go to: [{goal1}]")
    # drone_interface.go_to_point(goal1, speed=speed, ignore_yaw=True)
    # print("Go to done")

    # sleep(1.0)
    # print(f"Go to: [{goal2}]")
    # drone_interface.go_to_point(goal2, speed=speed, ignore_yaw=True)
    # print("Go to done")

    # ##### FOLLOW PATH #####
    # path = [goal0, goal1, goal2]
    # drone_interface.follow_path(path, speed=speed)

    sleep(1.0)
    print(f"Land: [{goal2}]")
    drone_interface.land(speed=0.5)
    print("Land done")

    print("Clean exit")


if __name__ == '__main__':
    rclpy.init()
    drone = DroneInterface("tello", verbose=True)

    drone_run(drone)

    drone.shutdown()
    rclpy.shutdown()
    sys.exit(0)
