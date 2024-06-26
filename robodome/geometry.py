"""
This script contains a geometry generator to generate the correct waypoint locations for the robodome project
"""
import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos
import bisect
from scipy.interpolate import interp1d

from typing import Tuple, List

def euler_to_rotation_matrix(roll, pitch, yaw):
    R_x = np.array([[1, 0, 0],
                    [0, cos(roll), -sin(roll)],
                    [0, sin(roll), cos(roll)]])
    R_y = np.array([[cos(pitch), 0, sin(pitch)],
                    [0, 1, 0],
                    [-sin(pitch), 0, cos(pitch)]])
    R_z = np.array([[cos(yaw), -sin(yaw), 0],
                    [sin(yaw), cos(yaw), 0],
                    [0, 0, 1]])
    
    return np.dot(R_x, np.dot(R_y, R_z))

class Tier(object):

    def __init__(self, height:float, radius:float):
        self.height = height
        self.radius = radius

class TieredGeometry(object):

    def __init__(self):

        self.transform: np.ndarray = np.eye(4)
        self.swap_matrix: np.ndarray = np.eye(4)

        self.tiers:List[Tier] = []

    def set_transform(self, x:float, y:float, z:float, roll: float, pitch: float, yaw:float, scale:float=1):

        translation_vector = np.array([x, y, z])

        scale_matrix = np.eye(3) * scale
        rotation_matrix = euler_to_rotation_matrix(roll, pitch, yaw)

        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = np.dot(scale_matrix, rotation_matrix)
        transformation_matrix[:3, 3] = translation_vector

        self.transform = transformation_matrix
    
    def set_swap_axis(self, axis_1:int, axis_2:int):
        swap_matrix = np.eye(4)
        swap_matrix[:, [axis_1, axis_2]] = swap_matrix[:, [axis_2, axis_1]]
        self.swap_matrix = swap_matrix

    def __transform_point(self, point:np.ndarray) -> np.ndarray:
        point = np.append(point, 1)
        transform = np.dot(np.dot(self.transform, self.swap_matrix), point)
        trans_point = transform[:3] / transform[3]
        return trans_point
    
    def add_tier(self, tier: Tier):
        # Insert in height order to ensure self.tiers is sorted by height
        bisect.insort(self.tiers, tier, key=lambda x: x.height)
    
    def get_tiered_points(self, points_per_level:int) -> np.ndarray[np.float32]:
        points = []
        for tier in self.tiers:
            angles = np.linspace(0, 2* np.pi, points_per_level, endpoint=False)
            xs = tier.radius * np.cos(angles)
            ys = tier.radius * np.sin(angles)
            zs = np.array([tier.height for _ in range(points_per_level)])
            trans_points = [self.__transform_point(np.array([x, y, z])) for x, y, z in zip(xs, ys, zs) ]
            points.extend(trans_points)

        return np.array(points)

    def get_points(self, num_levels:int, points_per_level:int, interp_method="linear") -> np.ndarray[np.float32]:
        # Generate the corresponding x values for the input heights
        height_values = [t.height for t in self.tiers]
        radius_values = [t.radius for t in self.tiers]

        # Create an interpolation function
        interp_func = interp1d(height_values, radius_values, kind=interp_method)

        # Generate n evenly spaced x values for the resampled heights
        resampled_height_values = np.linspace(min(height_values), max(height_values), num_levels)

        # Compute the corresponding resampled heights using the interpolation function
        resampled_radii = interp_func(resampled_height_values)

        points = []
        for h, r in zip(resampled_height_values, resampled_radii):
            angles = np.linspace(0, 2* np.pi, points_per_level, endpoint=False)
            xs = r * np.cos(angles)
            ys = r * np.sin(angles)
            zs = np.array([h for _ in range(points_per_level)])
            trans_points = [self.__transform_point(np.array([x, y, z])) for x, y, z in zip(xs, ys, zs) ]
            points.extend(trans_points)
        return np.array(points)
    
    def get_point(self, height:float, angle:float, interp_method="linear") -> np.ndarray[np.float32]:
        # Generate the corresponding x values for the input heights
        height_values = [t.height for t in self.tiers]
        radius_values = [t.radius for t in self.tiers]

        if height < min(height_values):
            height = min(height_values)
        elif height > max(height_values):
            height = max(height_values)

        interp_func = interp1d(height_values, radius_values, kind=interp_method)
        radius_at_height = interp_func(height)

        x = radius_at_height * np.cos(angle)
        y = radius_at_height * np.sin(angle)
        z = radius_at_height

        return self.__transform_point(np.array[x, y, z])

    def points_append_yaw(self, points: np.ndarray) -> np.ndarray:
        origin = self.transform[:3, 3]
        out_p = []
        for p in points:
            rel_pos = origin - p
            yaw = np.arctan2(rel_pos[1], rel_pos[0]) 
            out = np.append(p, yaw)
            out_p.append(out)
        return out_p
    
    def plot(self, show=True):

        def triplets_to_arrays(points):
            points = np.array(points)
            x_coords = points[:, 0]
            y_coords = points[:, 1]
            z_coords = points[:, 2]
            return x_coords, y_coords, z_coords
        
        # Generate points for both functions
        x1, y1, z1 = triplets_to_arrays(self.get_tiered_points(points_per_level=10))
        x2, y2, z2 = triplets_to_arrays(self.get_points(num_levels=10, points_per_level=10))

        # Plot the points on separate subplots
        fig, axs = plt.subplots(1, 2, subplot_kw={'projection': '3d'})

        # Plot for function 1
        axs[0].plot(x1, y1, z1, 'xr')
        axs[0].set_title('Tiered Points\n(10 ppl)')
        axs[0].set_xlabel('X')
        axs[0].set_ylabel('Y')
        axs[0].set_zlabel('Z')

        # Plot for function 2
        axs[1].plot(x2, y2, z2, 'xb')
        axs[1].set_title('Interpolated Points\n(10 levels, 10 ppl)')
        axs[1].set_xlabel('X')
        axs[1].set_ylabel('Y')
        axs[1].set_zlabel('Z')

        if show:
            plt.show()

class GeometryBuilder(object):
    
    def __init__(self):
        self.hemisphere = TieredGeometry()

    def add_tier(self, height: float, radius: float):
        self.hemisphere.add_tier(Tier(height, radius))

    def build(self) -> TieredGeometry:
        return self.hemisphere



def generate_R300Dome() -> TieredGeometry:
    hb = GeometryBuilder()
    hb.add_tier(30, 300)
    hb.add_tier(85, 290)
    hb.add_tier(140, 260)
    hb.add_tier(200, 210)
    hb.add_tier(260, 110)
    return hb.build()

def generate_R150Dome() -> TieredGeometry:
    hb = GeometryBuilder()
    hb.add_tier(30, 150)
    hb.add_tier(85, 130)
    hb.add_tier(150, 70)
    # hb.add_tier(200, 210)
    # hb.add_tier(260, 110)
    return hb.build()




if __name__ == "__main__":

    tiered_geom = generate_R300Dome()
    tiered_geom.plot()