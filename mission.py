#!/bin/python3

""" Mission example """

from time import sleep
import rclpy
from as2_python_api.drone_interface import DroneInterface


def drone_run(drone_interface: DroneInterface):
    """ Run the mission """

    speed = 0.5
    takeoff_height = 1.0
    height = 1.0

    sleep_time = 2.0

    dim = 0.5
    # path = [
    #     [-dim, dim, height],
    #     [-dim, -dim, height],
    #     [dim, -dim, height],
    #     [dim, dim, height]
    # ]
    path = [
        [dim, 0, height]
    ]
    print("Start mission")

    ##### ARM OFFBOARD #####
    print("Arm")
    drone_interface.offboard()
    sleep(sleep_time)
    print("Offboard")
    drone_interface.arm()
    sleep(sleep_time)

    ##### TAKE OFF #####
    print("Take Off")
    drone_interface.takeoff(takeoff_height, speed=1.0)
    print("Take Off done")
    sleep(sleep_time)
    sleep(sleep_time)

    ##### GO TO #####
    for goal in path:
        print(f"Go to with path facing {goal}")
        drone_interface.go_to.go_to_point_path_facing(goal, speed=speed)
        print("Go to done")
    sleep(sleep_time)

    ##### LAND #####
    print("Landing")
    drone_interface.land(speed=0.5)
    print("Land done")

    drone_interface.disarm()


def main():
    """ Main """
    rclpy.init()

    uav = DroneInterface(drone_id="tello", verbose=False, use_sim_time=False)

    drone_run(uav)

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")


if __name__ == '__main__':
    main()
