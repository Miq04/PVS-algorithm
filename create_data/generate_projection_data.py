import os
import pickle
import pandas as pd
import numpy as np
from matplotlib.path import Path
import matplotlib.pyplot as plt
from concurrent.futures import ThreadPoolExecutor

from LiDAR.Lidar import LiDARTypes
from LiDAR.LidarPosition import LIDARPosition
from LiDAR.LidarPointCloudModel import simulate_lidar_points

# 定义分块大小
CHUNK_SIZE = 2000
_INPUT_PATH = r"C:\Users\hx01\WPSDrive\267329050\WPS云盘\研究生\0.毕业论文\code\lidar\data\202403201305"

def write_pickle(obj, path):
    with open(path, 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

def read_pickle(path):
    with open(path, 'rb') as f:
        return pickle.load(f)

def save_projection_data_to_disk(projection_data, chunk_number, output_directory):
    filepath = os.path.join(output_directory, f'projection_data_{chunk_number}.pkl')
    with open(filepath, 'wb') as file:
        pickle.dump(projection_data, file)

def split_by_time(data, time_start, span):
    data = data[(data['t'] >= time_start) & (data['t'] < time_start + span)]
    return data.sort_values(by=['t','veh_id'])

def vehicle_position(data, t, veh_id):
    row = data[(data['t'] == t) & (data['veh_id'] == veh_id)]
    if not row.empty:
        return row.iloc[0]['veh_x'], row.iloc[0]['veh_y']
    else:
        print(f"Vehicle {veh_id} not found at time {t}.")
        return None

def is_point_in_polygon(point, polygon):
    path = Path(polygon)
    return path.contains_point(point)

def calculate_polygon_area(polygon):
    x = [p[0] for p in polygon]
    y = [p[1] for p in polygon]
    return 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))

def calculate_lane_coverage(point_cloud_array, lane_dict):
    lane_coverage = {lane: set() for lane in lane_dict}
    unique_points = set(tuple(point[:2]) for point in point_cloud_array)
    
    for point in unique_points:
        for lane, polygon in lane_dict.items():
            if is_point_in_polygon(point, polygon):
                lane_coverage[lane].add(point)
                break

    return lane_coverage

def generate_particular_vehicle_point_clouds_projection(data_mapping, t, veh_id, lidar, lidar_pos, lane_dict):
    row = data_mapping[(data_mapping['t'] == t) & (data_mapping['veh_id'] == veh_id)]
    if not row.empty:
        veh_x, veh_y, veh_len, veh_wid, veh_hig = row.iloc[0]['veh_x'], row.iloc[0]['veh_y'], row.iloc[0]['veh_len'], row.iloc[0]['veh_wid'], row.iloc[0]['veh_hig']
        pc_arary = simulate_lidar_points(lidar, veh_x, veh_y, veh_len, veh_wid, veh_hig, lidar_pos)
        return calculate_lane_coverage(pc_arary, lane_dict)
    else:
        print(f"Vehicle {veh_id} not found at time {t}.")
        return np.array([])

def generate_projection_for_all_vehicles(data_mapping, lidar, lidar_pos, lane_dict):
    time_veh_id_combinations = data_mapping[['t', 'veh_id']].drop_duplicates()
    
    projection_data = {}
    with ThreadPoolExecutor(max_workers=8) as executor:
        future_to_veh_id = {executor.submit(generate_particular_vehicle_point_clouds_projection, data_mapping, row['t'], row['veh_id'], lidar, lidar_pos, lane_dict):
                            (row['t'], row['veh_id']) for _, row in time_veh_id_combinations.iterrows()}
        
        for future in future_to_veh_id:
            t, veh_id = future_to_veh_id[future]
            try:
                projection = future.result()
                count = sum(1 for value in projection.values() if len(value))
                if count > 0:
                    projection_data[(t, veh_id)] = projection
            except Exception as e:
                print(f"Error generating projection for vehicle {veh_id} at time {t}: {e}")
    
    return projection_data

def generate_and_save_projection_for_all_vehicles(data_mapping, lidar, lidar_pos, lane_dict, output_directory):
    time_veh_id_combinations = data_mapping[['t', 'veh_id']].drop_duplicates()

    current_chunk_data = {}
    current_chunk_number = 0
    total_count = 0
    
    with ThreadPoolExecutor(max_workers=10) as executor:
        future_to_veh_id = {executor.submit(generate_particular_vehicle_point_clouds_projection, data_mapping, row['t'], row['veh_id'], lidar, lidar_pos, lane_dict):
                            (row['t'], row['veh_id']) for _, row in time_veh_id_combinations.iterrows()}

        for future in future_to_veh_id:
            t, veh_id = future_to_veh_id[future]
            try:
                projection = future.result()
                count = sum(1 for value in projection.values() if len(value))
                if count > 0:
                    current_chunk_data[(t, veh_id)] = projection
                    total_count += 1
                    
                    # 每达到一定数量的数据项，就保存到磁盘
                    if total_count % CHUNK_SIZE == 0:
                        save_projection_data_to_disk(current_chunk_data, current_chunk_number, output_directory)
                        current_chunk_data = {}
                        current_chunk_number += 1
                        
            except Exception as e:
                print(f"Error generating projection for vehicle {veh_id} at time {t}: {e}")

        # 处理剩余不足一个chunk的数据
        if current_chunk_data:
            save_projection_data_to_disk(current_chunk_data, current_chunk_number, output_directory)

def main():
    data = pd.read_csv(f'{_INPUT_PATH}/sumoTrace_veh_type.csv', index_col=None)
    if 'Unnamed: 0' in data.columns:
        del data['Unnamed: 0']

    data_split = split_by_time(data, 24*60, 10*60)

    # lane 映射
    unique_lanes = sorted(data_split['veh_lane'].unique())
    lane_mapping = {lane: idx for idx, lane in enumerate(unique_lanes, 1)}
    data_mapping = data_split.copy()
    data_mapping['veh_lane'] = data_mapping['veh_lane'].map(lane_mapping)

    # save temp
    write_pickle(lane_mapping, f"{_INPUT_PATH}/lane_mapping.pkl")
    data_mapping.to_csv(f'{_INPUT_PATH}/data_mapping.csv', index=False)

    lidar = LiDARTypes.Test
    lidar_pos = LIDARPosition.FRONT_CENTER
    lane_dict = read_pickle("data/pkl/lane_dict.pkl")

    # ------------------test------------------
    test_t = data_mapping.iloc[0, 0]
    test_veh_id = data_mapping.iloc[0, 1]
    lane_projection = generate_particular_vehicle_point_clouds_projection(data_mapping, test_t, test_veh_id, lidar, lidar_pos, lane_dict)
    non_empty_keys_count = sum(1 for value in lane_projection.values() if len(value))
    print("non_empty_keys_count",non_empty_keys_count)

    # ------------------generate projection data------------------
    print("Projection Data Generating...")
    projection_data = generate_and_save_projection_for_all_vehicles(data_mapping, lidar, lidar_pos, lane_dict, _INPUT_PATH)

    # ------------------check data quality------------------
    lane_point_totals = {}
    time_veh_id_combinations = data_mapping[['t', 'veh_id']].drop_duplicates()

    for index, row in time_veh_id_combinations.iterrows():
        t = row['t']
        veh_id = row['veh_id']

        data_key = (t, veh_id)
        if data_key in projection_data:
            lane_point_mapping = projection_data[data_key]

            for lane, point in lane_point_mapping.items():
                if lane in lane_point_totals:
                    lane_point_totals[lane] += len(point)
                else:
                    lane_point_totals[lane] = len(point)


    lane_labels = sorted(lane_point_totals.keys())
    lane_values = [lane_point_totals[lane] for lane in lane_labels]

    plt.figure(figsize=(12, 6))
    plt.bar(lane_labels, lane_values, align='center')
    plt.xlabel('Lane Identifier')
    plt.ylabel('Number of Points')
    plt.title('Point Totals by Lane')
    plt.xticks(rotation=45)
    plt.tight_layout()

    plt.savefig('point_totals_by_lane.png')
    plt.show()


if __name__ == "__main__":
    main()