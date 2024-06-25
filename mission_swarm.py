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
from as2_msgs.msg import YawMode
from as2_msgs.msg import BehaviorStatus
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler


class Choreographer:
    """Simple Geometric Choreographer"""

    @staticmethod
    def delta_formation(base: float, height: float, orientation: float = 0.0, center: list = [0.0, 0.0]):
        """Triangle"""
        theta = radians(orientation)
        v0 = [-height * cos(theta) / 2.0 - base * sin(theta) / 2.0 + center[0],
              base * cos(theta) / 2.0 - height * sin(theta) / 2.0 + center[1]]
        v1 = [height * cos(theta) / 2.0 + center[0], height * sin(theta) / 2.0 + center[1]]
        v2 = [-height * cos(theta) / 2.0 + base * sin(theta) / 2.0 + center[0],
              -base * cos(theta) / 2.0 - height * sin(theta) / 2.0 + center[1]]
        return [v0, v1, v2]

    @staticmethod
    def line_formation(length: float, orientation: float = 0.0, center: list = [0.0, 0.0]):
        """Line"""
        theta = radians(orientation)
        l0 = [length * cos(theta) / 2.0 + center[1], length * sin(theta) / 2.0 + center[1]]
        l1 = [0.0 + center[1], 0.0 + center[1]]
        l2 = [-length * cos(theta) / 2.0 + center[1], -length * sin(theta) / 2.0 + center[1]]
        return [l0, l1, l2]

    @staticmethod
    def draw_waypoints(waypoints):
        """Debug"""
        import matplotlib.pyplot as plt

        print(waypoints)

        xaxys = []
        yaxys = []
        for wp in waypoints:
            xaxys.append(wp[0])
            yaxys.append(wp[1])
        plt.plot(xaxys, yaxys, 'o-b')
        plt.xlim(-3, 3)
        plt.ylim(-3, 3)
        plt.ylabel('some numbers')
        plt.show()

    @staticmethod
    def do_cycle(formation: list, index: int, height: int):
        """List to cycle with height"""
        return list(e + [height]
                    for e in list(islice(cycle(formation), 0+index, 3+index)))


class Dancer(DroneInterface):
    """Drone Interface extended with path to perform and async behavior wait"""

    def __init__(self, namespace: str, path: list, verbose: bool = False,
                 use_sim_time: bool = False):
        super().__init__(namespace, verbose=verbose, use_sim_time=use_sim_time)

        self.__path = path

        self.__current = 0

        self.__speed = 0.3
        self.__yaw_mode = YawMode.PATH_FACING
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
        self.do_behavior("go_to", point[0], point[1], point[2], self.__speed,
                         self.__yaw_mode, self.__yaw_angle, self.__frame_id, False)
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
        for index, name in enumerate(drones_ns):
            path = get_path(index)
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
        for _ in range(len(get_path(0))):
            for drone in self.drones.values():
                drone.go_to_next()
            self.wait()


def get_path(i: int) -> list:
    """Path: initial, steps, final

    1   1           6       7           0
    2       2   5               8       1
    0   3           4       9           2

    """
    center = [2.0, -1.0]
    delta_frontward = Choreographer.delta_formation(2, 2, 0, center)
    delta_backward = Choreographer.delta_formation(2, 2, 180, center)
    line = Choreographer.line_formation(2, 180, center)

    h1 = 1.0
    h2 = 2.0
    h3 = 3.0
    line_formation = [line[i] + [h3]]
    return Choreographer.do_cycle(delta_frontward, i, h1) + \
        Choreographer.do_cycle(delta_backward, i, h2) + \
        Choreographer.do_cycle(delta_frontward, i, h3) + \
        line_formation


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

    drones_ns = ['tello3'] if len(args.drone_ns) == 0 else args.drone_ns
    print(f"Using Drones: {drones_ns}")

    if args.simulated:
        print("Mission running in simulation mode")
    else:
        print("Mission running in real mode")

    rclpy.init()
    swarm = SwarmConductor(drones_ns, verbose=False,
                         use_sim_time=args.simulated)

    try:
        if confirm("Takeoff"):
            swarm.get_ready()
            swarm.takeoff()

            if confirm("Go to"):
                swarm.dance()

                while confirm("Replay"):
                    swarm.dance()
            
            confirm("Land")
    except KeyboardInterrupt as e:
        pass
    finally:
        print("Landing")
        swarm.land()

    print("Shutdown")
    swarm.shutdown()
    rclpy.shutdown()

    sys.exit(0)


if __name__ == '__main__':
    main()
