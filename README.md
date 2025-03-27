├── analysis
│   ├── Model1.py              # 基于时间探测策略的优化模型
│   ├── Model2.py              # 基于时空探测策略的优化模型
│   └── Model3.py              # 基于加权时空探测策略的优化模型
├── area_shape
│   └── JunctionArea.py        # 交叉口区域定义及计算
├── create_data
│   ├── generate_flow_dataset.py    # 生成流量数据集
│   ├── generate_projection_data.py # 生成投影数据
│   └── sumotrace.sh               # SUMO轨迹数据生成脚本
├── preprocess
│   ├── DetectionGrids.py      # 车辆探测栅格生成与处理
│   └── GenerateMatrix.py      # 生成优化模型输入矩阵
├── sensor
│   ├── Lidar.py               # LiDAR传感器模拟
│   ├── LidarPointCloudModel.py  # LiDAR点云模型生成
│   ├── LidarPosition.py        # LiDAR位置与方向设置
│   └── calculate_attenuation.py # 计算LiDAR衰减
└── utils.py                   # 常用工具函数
