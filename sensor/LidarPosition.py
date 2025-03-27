from enum import Enum

class LIDARPosition(Enum):
    FRONT_CENTER = "front-center"
    FRONT_LEFT = "front-left"
    FRONT_RIGHT = "front-right"
    CENTER = "center"

def calculate_lidar_position(veh_x, veh_y, veh_len, veh_wid, veh_hig, lidar_pos):
    if lidar_pos == LIDARPosition.FRONT_CENTER:
        return veh_x, veh_y + veh_len / 2, veh_hig
    elif lidar_pos == LIDARPosition.FRONT_LEFT:
        return veh_x - veh_wid / 2, veh_y + veh_len / 2, veh_hig
    elif lidar_pos == LIDARPosition.FRONT_RIGHT:
        return veh_x + veh_wid / 2, veh_y + veh_len / 2, veh_hig
    elif lidar_pos == LIDARPosition.CENTER:
        return veh_x, veh_y, veh_hig
    else:
        raise ValueError("Invalid LIDAR position")


if __name__ == '__main__':
    veh_x = 0
    veh_y = 0
    veh_len = 4
    veh_wid = 2
    veh_hig = 1.5
    lidar_pos = LIDARPosition.FRONT_CENTER
    result = calculate_lidar_position(veh_x, veh_y, veh_len, veh_wid, veh_hig, lidar_pos)
    print(f"Sensor position for {lidar_pos.value}: {result}")
