import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from math import cos, sin, radians
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely.ops import cascaded_union
from LiDAR.Lidar import LiDARTypes
from LiDAR.LidarPosition import LIDARPosition, calculate_lidar_position
from utils import read_pickle


EPSILON = 1e-8


def vehicle_info(t, veh_id, data):
    row = data[(data['t'] == t) & (data['veh_id'] == veh_id)]
    if not row.empty:
        return row.iloc[0][[2,3,6]] # x y lane
    else:
        print(f"Vehicle {veh_id} not found at time {t}.")
        return None

def search_all_vehicle(t, data):
    all_vehicle_pos_at_t = []
    all_vehicle_at_t = data[data['t'] == t]
    if len(all_vehicle_at_t):
        for row in all_vehicle_at_t.itertuples():
            all_vehicle_pos_at_t.append((row.veh_x, row.veh_y))
    return all_vehicle_pos_at_t

def scale_round_coords(coords, precision=1):
    scale_factor = 10 ** precision
    return [(int(round(x * scale_factor)) / scale_factor, int(round(y * scale_factor)) / scale_factor) for x, y in coords]

class GridMap:
    def __init__(self, veh_id, t, data, grid_len, grid_width = 3.2):
        self.veh_id = veh_id
        self.t = t
        self.data = data
        self.grid_len = grid_len
        self.grid_width = grid_width
        self.load_veh_info(self.data)
        self.grids_direction()
    
    def load_veh_info(self, data):
        row = vehicle_info(self.t, self.veh_id, data)
        veh_x, veh_y, v_lane = row['veh_x'], row['veh_y'], row['veh_lane']
        self.veh_x = veh_x

        self.veh_y = veh_y
        self.v_lane = v_lane
    
    def grids_direction(self):
        # X - horizontal, Y - vertical
        if 'E0' in self.v_lane or 'E3' in self.v_lane:
            self.grids_direction = 'horizontal'
        elif 'E1' in self.v_lane or 'E2' in self.v_lane:
            self.grids_direction = 'vertical'

    def generate_grids(self, lidar, lane_dict):
        shape = [0]*4
        lidar_range = lidar.range_max
        if self.grids_direction == 'horizontal':
            shape[0] = (self.veh_x - lidar_range/2, self.veh_y + self.grid_width*1.5)
            shape[1] = (self.veh_x - lidar_range/2, self.veh_y - self.grid_width*1.5)
            shape[2] = (self.veh_x + lidar_range/2, self.veh_y - self.grid_width*1.5)
            shape[3] = (self.veh_x + lidar_range/2, self.veh_y + self.grid_width*1.5)
        elif self.grids_direction == 'vertical':
            shape[0] = (self.veh_x - self.grid_width*1.5, self.veh_y - lidar_range/2)
            shape[1] = (self.veh_x + self.grid_width*1.5, self.veh_y - lidar_range/2)
            shape[2] = (self.veh_x + self.grid_width*1.5, self.veh_y + lidar_range/2)
            shape[3] = (self.veh_x - self.grid_width*1.5, self.veh_y + lidar_range/2)
        else:
            raise ValueError("Invalid grid direction")
        print(scale_round_coords(shape))
        shape_polygon = Polygon(scale_round_coords(shape))
        polygons = [Polygon(scale_round_coords(coords)) for coords in lane_dict.values()]

        intersections = [shape_polygon.intersection(polygon.buffer(0)) for polygon in polygons]
        final_intersection = unary_union(intersections)

        self.grids = final_intersection
        print("最终的交集多边形的坐标:", list(final_intersection.exterior.coords))
        print("最终的交集多边形的面积:", final_intersection.area)

    def occlusion(self, lane_dict):
        pass


def simulate_lidar_points(lidar, veh_x, veh_y, veh_len, veh_wid, veh_hig, lidar_pos):
    sensor_pos = calculate_lidar_position(veh_x, veh_y, veh_len, veh_wid, veh_hig, lidar_pos)
    sensor_pos = np.array(sensor_pos,dtype=np.float32)
    points = []
    vertical_angles = np.linspace(lidar.vertical_fov[0], lidar.vertical_fov[1], lidar.num_channels)
    for angle_h in np.arange(0, lidar.horizontal_fov, lidar.horizontal_resolution):
        for angle_v in vertical_angles:
            direction = np.array([
                cos(radians(angle_v)) * sin(radians(angle_h)),
                cos(radians(angle_v)) * cos(radians(angle_h)),
                sin(radians(angle_v))
            ])
            # for r in np.linspace(lidar.range_min, lidar.range_max, int(lidar.range_max / (1 + abs(angle_v)/10))):
            for r in np.linspace(lidar.range_min, lidar.range_max, int((lidar.range_max-lidar.range_min)/10)):
                point = sensor_pos + direction + direction * r
                points.append(point)
    
    return np.array(points)


if __name__ == '__main__':
    # ------------------lidar simulation------------------
    veh_x, veh_y, veh_len, veh_wid, veh_hig = 0, 0, 4, 2, 1.5
    points = simulate_lidar_points(LiDARTypes.Test1, veh_x, veh_y, veh_len, veh_wid, veh_hig, LIDARPosition.FRONT_CENTER)
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:,2], s=1)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.axis('equal')
    plt.title('LiDAR Coverage Area')
    plt.show()

    # ------------------grids simulation------------------
    # data = pd.read_csv('data/202403291546/sumoTrace_veh_type.csv')
    # del data['Unnamed: 0']
    # lane_dict = read_pickle('data/pkl/lane_dict.pkl')
    # test_t = data.iloc[0, 0]
    # test_veh_id = data.iloc[0, 1]
    # test_grids = GridMap(test_veh_id, test_t, data, 4.6)
    # test_grids.generate_grids(LiDARTypes.Test1, lane_dict)
