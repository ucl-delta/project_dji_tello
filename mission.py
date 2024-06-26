#!/bin/python3

""" Mission example """
import argparse
from time import sleep
import rclpy
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.modules.motion_reference_handler_module import MotionReferenceHandlerModule


def drone_run(drone_interface: DroneInterface):
    """ Run the mission """

    center = 1
    speed = 0.5
    takeoff_height = 1.0
    height = 1.0

    sleep_time = 2.0

    dim = 0.5
    path = [
        [center-dim, center+dim, height],
        [center-dim, center-dim, height],
        [center+dim, center-dim, height],
        [center+dim, center+dim, height]
    ]
    # path = [
    #     [dim, 0, height]
    # ]
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

    while True:
        inp = input("Where to go (type 'land' to stop): [x y z] >")
        if inp == "land": 
            break

        print(f"Going to {inp}")
        try:
            x, y, z = inp.strip().split(" ")
            drone_interface.go_to.go_to_point([x, y, z], speed=speed)
        except Exception as e:
            print("Something went wrong:", e)
    
    # ##### GO TO #####
    # for _ in range(3):
    #     for goal in path:
    #         print(f"Go to with path facing {goal}")
    #         drone_interface.go_to.go_to_point_path_facing(goal, speed=speed)
    #         print("Go to done")
    # sleep(sleep_time)

    ##### LAND #####
    print("Landing")
    drone_interface.land(speed=0.5)
    print("Land done")

    drone_interface.disarm()


def main():

    parser = argparse.ArgumentParser(
        description="Manually fly tello around")
    parser.add_argument('namespace', type=str, default="tello1")

    args = parser.parse_args()

    """ Main """
    rclpy.init()

    uav = DroneInterface(drone_id=args.namespace, verbose=False, use_sim_time=False)

    try:
        drone_run(uav)
    except KeyboardInterrupt as e:
        print("Keyboard Interrupt Detected, Landing")
        uav.land(speed=0.5)
        uav.disarm()
    finally:
        uav.shutdown()

    rclpy.shutdown()

    print("Clean exit")


if __name__ == '__main__':
    main()
