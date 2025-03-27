import pandas as pd
from shapely.geometry import Point, Polygon
import concurrent.futures
from tqdm import tqdm
from utils import read_pickle, write_pickle


def split_by_time(data, time_start, span):
    data = data[(data['t'] >= time_start) & (data['t'] < time_start + span)]
    return data.sort_values(by=['t','veh_id'])

def get_direction(lane):
    # X - horizontal, Y - vertical
    if 'E0' in lane or 'E3' in lane:
        return 'horizontal'
    elif 'E1' in lane or 'E2' in lane:
        return 'vertical'
    raise ValueError(f"Invalid lane:{lane}")

def create_grid_polygon(grid_points):
    return Polygon(grid_points)

def preprocess_grids_dict(grids_dict):
    preprocessed_grids = {}

    for grid_id, grid_points in grids_dict.items():
        left, bottom = min(point[0] for point in grid_points), min(point[1] for point in grid_points)
        right, top = max(point[0] for point in grid_points), max(point[1] for point in grid_points)
        center = (round((left + right) / 2, 1), round((bottom + top) / 2, 1))
        direction = get_direction(grid_id)

        preprocessed_grids[grid_id] = {
            "polygon": create_grid_polygon(grid_points),
            "left": left,
            "bottom": bottom,
            "right": right,
            "top": top,
            "center": center,
            "direction": direction,
        }

    return preprocessed_grids

def intersects_circle(circle_center, circle_radius, left, right, bottom, top):
    cx, cy = circle_center
    cr = circle_radius

    # 检查圆心是否在矩形内或圆与矩形边界相交
    return (left - cr <= cx <= right + cr) and (bottom - cr <= cy <= top + cr) or (
        (cx - left <= cr) or (cx - right >= -cr) or 
        (cy - bottom <= cr) or (cy - top >= -cr)
    )

def find_covered_grids(circle_center, circle_radius, preprocessed_grids, threshold=0.5):
    covered_grid_ids = []
    grid_area = 4.6 * 3.2

    for grid_id, grid_info in preprocessed_grids.items():
        polygon = grid_info["polygon"]
        left, bottom, right, top = grid_info["left"], grid_info["bottom"], grid_info["right"], grid_info["top"]

        # 快速过滤掉明显不与圆相交的网格
        if not intersects_circle(circle_center, circle_radius, left, right, bottom, top):
            continue

        detected_area = polygon.intersection(Point(circle_center).buffer(circle_radius)).area
        if detected_area / grid_area >= threshold:
            covered_grid_ids.append(grid_id)

    return covered_grid_ids

grids_dict = read_pickle('data/pkl/grids_dict.pkl')
preprocessed_grids_dict = preprocess_grids_dict(grids_dict)
def calculate_covered_grids(row, circle_radius=50, preprocessed_grids_dict=preprocessed_grids_dict):
    t, veh_id, veh_x, veh_y, _ = row
    circle_center = (veh_x, veh_y)
    covered_grids = find_covered_grids(circle_center, circle_radius, preprocessed_grids_dict)
    return (t, veh_id), covered_grids

def parallel_calculate_covered_grids(df, max_workers=None):
    result_dict = {}

    with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as executor:
        future_to_row = {executor.submit(calculate_covered_grids, row): row for _, row in df.iterrows()}
        for future in tqdm(concurrent.futures.as_completed(future_to_row), total=len(df)):
            row = future_to_row[future]
            try:
                (t, veh_id), covered_grids = future.result()
                result_dict[(t, veh_id)] = covered_grids
            except Exception as exc:
                print(f"Error calculating covered grids for row {row}: {exc}")

    return result_dict


trajectory = pd.read_csv('data/20240409202305/trajectory_pre.csv')
trajectory = trajectory[trajectory['t'] >= 1440]
result_dict = parallel_calculate_covered_grids(trajectory, max_workers=16)
# write_pickle(result_dict, "data/pkl/20/covered_grids_pre_20.pkl")
detection_radius = 50
write_pickle(result_dict, f"data/pkl/{detection_radius}/covered_grids_pre_{detection_radius}.pkl")
