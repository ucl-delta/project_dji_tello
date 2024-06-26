#!/bin/python3

"""
mission_swarm.py
"""

import sys
import argparse
from typing import List, Optional
from math import radians, cos, sin
from itertools import cycle, islice
import rclpy
import numpy as np

from as2_msgs.msg import YawMode
from as2_msgs.msg import BehaviorStatus
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler

from robodome.geometry import generate_R300Dome, generate_R150Dome, GeometryBuilder


def plot_paths(paths):
    import matplotlib.pyplot as plt

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot each path
    for i, path in enumerate(paths):
        path = np.array(path)
        x, y, z = path[:, 0], path[:, 1], path[:, 2]
        ax.plot(x, y, z, label=f'Agent {i+1}')

        for x, y, z, yaw in path:
            dx = 0.2*np.cos(yaw)
            dy = 0.2*np.sin(yaw)
            ax.plot([x, x-dx], [y, y-dy], [z, z], 'r-')


    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    # plt.show()
    plt.show(block=False)


class Dancer(DroneInterface):
    """Drone Interface extended with path to perform and async behavior wait"""

    def __init__(self, namespace: str, path: list, verbose: bool = False,
                 use_sim_time: bool = False):
        super().__init__(namespace, verbose=verbose, use_sim_time=use_sim_time)

        self.__path = path

        self.__current = 0

        self.__speed = 0.5
        self.__yaw_mode = YawMode.FIXED_YAW
        self.__yaw_angle = None
        self.__frame_id = "earth"

        self.current_behavior: Optional[BehaviorHandler] = None

    def reset(self) -> None:
        """Set current waypoint in path to start point"""
        self.__current = 0

    def do_behavior(self, beh, *args) -> None:
        """Start behavior and save current to check if finished or not"""
        self.current_behavior = getattr(self, beh)
        self.current_behavior(*args)

    def go_to_next(self) -> None:
        """Got to next position in path"""
        point = self.__path[self.__current]
        print(f"{self.namespace} going to {point}")

        if len(point) > 3:
            yaw_angle = point[3]
        else:
            yaw_angle = self.__yaw_angle

        self.do_behavior("go_to", point[0], point[1], point[2], self.__speed,
                         self.__yaw_mode, yaw_angle, self.__frame_id, False)
        self.__current += 1

    def goal_reached(self) -> bool:
        """Check if current behavior has finished"""
        if not self.current_behavior:
            return False

        if self.current_behavior.status == BehaviorStatus.IDLE:
            return True
        return False


class SwarmConductor:
    """Swarm Conductor"""

    def __init__(self, drones_ns: List[str], verbose: bool = False,
                 use_sim_time: bool = False):
        self.drones: dict[int, Dancer] = {}


        # Allocate paths
        center = [0.0, -3.0]
        height = 2.0

        radius = 150 # cm
        radius_change = 50
        distance_between_tiers = 100 #cm

        hb = GeometryBuilder()
        for i, _ in enumerate(drones_ns):
            hb.add_tier( (i+1) * distance_between_tiers, radius + radius_change*i)

        geom = hb.build()
        geom.set_transform(center[0], center[1], height, 0, np.pi/2, np.pi/2, scale=0.01)
        geom.set_swap_axis(1, 2) # Swap y and z for ros. Z is horizontal and y is up.

        points_per_layer = 6
        waypoints_per_layer = geom.get_tiered_points(points_per_level=points_per_layer, separate_per_tier=True)

        point_allocation = []
        for i, wpl in enumerate(waypoints_per_layer):
            wps = geom.points_append_yaw(wpl)
            wps = np.roll(wps, i, axis=0)
            point_allocation.append(wps)

        plot_paths(point_allocation)

        self.path_allocation = point_allocation

        for index, name in enumerate(drones_ns):
            path = point_allocation[index]
            self.drones[index] = Dancer(name, path, verbose, use_sim_time)

    def shutdown(self):
        """Shutdown all drones in swarm"""
        for drone in self.drones.values():
            drone.shutdown()

    def reset_point(self):
        """Reset path for all drones in swarm"""
        for drone in self.drones.values():
            drone.reset()

    def wait(self):
        """Wait until all drones has reached their goal (aka finished its behavior)"""
        all_finished = False
        while not all_finished:
            all_finished = True
            for drone in self.drones.values():
                all_finished = all_finished and drone.goal_reached()

    def get_ready(self):
        """Arm and offboard for all drones in swarm"""
        for drone in self.drones.values():
            drone.arm()
            drone.offboard()

    def takeoff(self):
        """Takeoff swarm and wait for all drones"""
        for drone in self.drones.values():
            drone.do_behavior("takeoff", 1, 0.7, False)
        self.wait()

    def land(self):
        """Land swarm and wait for all drones"""
        for drone in self.drones.values():
            drone.do_behavior("land", 0.4, False)
        self.wait()

    def dance(self):
        """Perform swarm choreography"""
        self.reset_point()
        for _ in range(max( [len(p) for p in self.path_allocation] )):
            for drone in self.drones.values():
                drone.go_to_next()
            self.wait()


def confirm(msg: str = 'Continue') -> bool:
    """Confirm message"""
    confirmation = input(f"{msg}? (y/n): ")
    if confirmation == "y":
        return True
    return False


def main():
    """Entrypoint"""
    
    parser = argparse.ArgumentParser(
        description="Starts gates mission for crazyswarm in either simulation or real environment")
    parser.add_argument('-s', '--simulated',
                        action='store_true', default=False)
    parser.add_argument('--drone_ns', metavar='S', type=str, nargs='+', 
                        help='a list of namespaces to consider')

    args = parser.parse_args()

    drones_ns = ['tello1'] if len(args.drone_ns) == 0 else args.drone_ns
    print(f"Using Drones: {drones_ns}")

    if args.simulated:
        print("Mission running in simulation mode")
    else:
        print("Mission running in real mode")

    rclpy.init()
    swarm = SwarmConductor(drones_ns, verbose=False,
                         use_sim_time=args.simulated)

    if confirm("Takeoff"):
        swarm.get_ready()
        swarm.takeoff()

        if confirm("Go to"):
            swarm.dance()

            while confirm("Replay"):
                swarm.dance()

        confirm("Land")
        swarm.land()

    print("Shutdown")
    swarm.shutdown()
    rclpy.shutdown()

    sys.exit(0)


if __name__ == '__main__':
    main()
