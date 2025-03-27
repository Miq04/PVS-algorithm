import pandas as pd
import numpy as np
from utils import read_pickle, write_pickle

# data 
entry_exit = pd.read_csv("data/csv/entry_exit.csv")
trajectory = pd.read_csv("data/csv/entry_exit_pre.csv")
grids_dict = read_pickle('data/pkl/grids_dict.pkl')

# covered_dict
detection_radius = 20
# covered_dict = read_pickle("data/pkl/100/covered_grids_pre.pkl")
covered_dict = read_pickle(f"data/pkl/{detection_radius}/covered_grids_pre_{detection_radius}.pkl")

# matrix
T_start = 1440
C_duration = 30
grid_ids = list(grids_dict.keys())
num_grids = len(grid_ids)

# 初始化探测矩阵和Injunction矩阵
detection_matrices_list = []
in_junction_matrices_list = []

for i, t_start in enumerate(range(1440, 1440 + 20 * C_duration, C_duration)):
    # 获取当前时间窗口内的车辆列表和数量
    vehicle_ids = trajectory[(trajectory['t_entry']<=(t_start + C_duration)) & (trajectory['t_exit'] > t_start)]['veh_id'].unique()
    # vehicle_ids = entry_exit[entry_exit['t_entry'].between(t_start, t_start + C_duration)]['veh_id'].unique()
    num_vehicles = len(vehicle_ids)

    # 初始化探测矩阵和Injunction矩阵
    detection_matrix = np.zeros((num_grids, num_vehicles, C_duration), dtype=int)
    in_junction_matrix = np.zeros((num_vehicles, C_duration), dtype=int)

    vehicle_mapping = {veh_id: j for j, veh_id in enumerate(vehicle_ids)}
    time_mapping = {t: t - t_start for t in range(t_start, t_start + C_duration)}

    # 填充探测矩阵
    for (t, veh_id), grids in covered_dict.items():
        if veh_id in vehicle_mapping and t in time_mapping:
            time_idx = time_mapping[t]
            vehicle_idx = vehicle_mapping[veh_id]
            for grid in grids:
                if grid in grid_ids:
                    grid_idx = grid_ids.index(grid)
                    detection_matrix[grid_idx, vehicle_idx, time_idx] = 1

    # 填充Injunction矩阵
    for t in range(t_start, t_start + C_duration):
        for j, veh_id in enumerate(vehicle_ids):
            entry_time, exit_time = trajectory[trajectory['veh_id'] == veh_id][['t_entry', 't_exit']].iloc[0]
            if entry_time <= t <= exit_time:
                in_junction_matrix[j, t - t_start] = 1

    # 将当前时间窗口的探测矩阵和Injunction矩阵添加到列表中
    detection_matrices_list.append(detection_matrix)
    in_junction_matrices_list.append(in_junction_matrix)


write_pickle(detection_matrices_list, f"data/pkl/{detection_radius}/detection_matrices_pre_{detection_radius}.pkl")
write_pickle(in_junction_matrices_list, f"data/pkl/{detection_radius}/in_junction_matrices_pre_{detection_radius}.pkl")
