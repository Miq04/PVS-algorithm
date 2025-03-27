import pandas as pd
import random
import os
import matplotlib.pyplot as plt
import argparse


class VehicleType:
    def __init__(self, name, length, width, height, ratio):
        self.name = name
        self.length = length
        self.width = width
        self.height = height
        self.ratio = ratio

def id_generator(data, vehicle_type):
    num = data['veh_id'].nunique()  
    v_list = data['veh_id'].unique().tolist()
    v_list_type = random.sample(v_list, int(num * vehicle_type.ratio))
    v_list = [x for x in v_list if x not in v_list_type]
    return v_list, v_list_type

def label(data, v_list, vehicle_type):
    data_type = pd.DataFrame()
    for x in v_list:
        data_type = pd.concat([data_type, data.loc[data['veh_id'] == x]], ignore_index=True)
    data_type['veh_type'] = vehicle_type.name
    data_type['veh_len'] = vehicle_type.length
    data_type['veh_wid'] = vehicle_type.width
    data_type['veh_hig'] = vehicle_type.height
    return data_type.drop_duplicates()

def split_by_time(data, time_start, span):
    data = data[(data['t'] >= time_start) & (data['t'] < time_start + span)]
    return data.sort_values(by=['t','veh_id'])

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-path', type=str, required=True, help='The path to the input SumoTrace.csv')
    args = parser.parse_args()

    input_path = args.input_path
    data_path = os.path.join(input_path, "sumoTrace.csv")
    data = pd.read_csv(data_path, sep=';')
    data = data.dropna()

    # data.rename(columns={"Global_Time":"t","Vehicle_ID":"veh_id","Local_X":"veh_x","Local_Y":"veh_y",
    #                                 "v_angle":"veh_ang","v_Lane":"veh_lane","v_Vel":"veh_vel","v_type":"veh_type",
    #                                 "v_Length":"veh_len","v_Width":"veh_wid"},inplace=True)
    # data['veh_lane'] = data.loc[:, 'veh_lane'].map(lambda x: x.lstrip(':').rstrip('aAbBcC'))
    data.rename(columns={
        "timestep_time": "t",
        "vehicle_id": "veh_id",
        "vehicle_x": "veh_x",
        "vehicle_y": "veh_y",
        "vehicle_angle": "veh_ang",
        "vehicle_lane": "veh_lane",
        "vehicle_speed": "veh_vel",
        "vehicle_type": "veh_type",
    }, inplace=True)
    data['veh_lane'] = data.loc[:, 'veh_lane'].map(lambda x: x.lstrip(':'))

    data1 = data[['t','veh_id','veh_x','veh_y','veh_vel','veh_ang','veh_lane']]
    vehicle_types = [
        VehicleType('car', 4.2, 1.6, 1.5, 0.8),
        VehicleType('bus', 12, 2.6, 3.5, 0.1),
        VehicleType('truck', 13, 2.5, 4, 0.1),
    ]
    assert sum(vehicle_type.ratio for vehicle_type in vehicle_types) == 1, "Sum of ratios must be equal to 1"

    data_p = pd.DataFrame()
    for vehicle in vehicle_types:
        _, v_list_type = id_generator(data, vehicle)
        data_type = label(data1, v_list_type, vehicle)
        data_p = pd.concat([data_p, data_type], ignore_index=True)

    data_p = data_p.sort_values("veh_id")
    if 'Unnamed: 0' in data_p.columns:
        del data_p['Unnamed: 0']

    data_split = split_by_time(data_p, 24*60, 10*60)
    data_split.to_csv(f'{input_path}/sumoTrace_veh_type.csv')
    selected_col = ['t', 'veh_id', 'veh_type', 'veh_x', 'veh_y', 'veh_lane']
    data_split1 = data.loc[:, selected_col]
    data_split1.to_csv(f'{input_path}/sumoTrace_p.csv')
    value_counts = data_split['veh_lane'].value_counts()
    counts = pd.Series(value_counts)

    plt.figure(figsize=(10, 6))
    plt.bar(counts.index, counts.values) 
    plt.xlabel('Lane Label')
    plt.ylabel('Frequency')
    plt.title('Vehicle Distribution by Lane')
    plt.xticks(rotation=90)
    plt.tight_layout()
    plt.savefig(f"{input_path}/vehicle_distribution_by_lane.png", dpi = 300)
    plt.show()


if __name__ == "__main__":
    main()
