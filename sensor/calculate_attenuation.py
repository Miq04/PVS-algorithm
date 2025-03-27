import numpy as np
import os
import matplotlib.pyplot as plt

def load_velodyne_points(bin_file_path):
    points = np.fromfile(bin_file_path, dtype=np.float32).reshape(-1, 4)  # 每行4个值 [x, y, z, intensity]
    return points

# 计算点到激光雷达的距离
def calculate_distance(points):
    return np.sqrt(points[:, 0]**2 + points[:, 1]**2 + points[:, 2]**2)

# 根据指数衰减模型计算反射强度
def calculate_attenuation(points, alpha=0.01):
    # 选取第一个点的反射强度作为 I_0
    I_0 = points[0, 3]  
    distances = calculate_distance(points)
    intensities = I_0 * np.exp(-alpha * distances)  # 使用指数衰减模型计算强度
    return intensities

# 可视化反射强度与距离的关系
def plot_attenuation(points, calculated_intensities, file_name):
    distances = calculate_distance(points)
    intensities = points[:, 3]
    
    plt.scatter(distances, intensities, color='blue', label='Measured Intensities')
    plt.plot(distances, calculated_intensities, color='red', label='Calculated Intensities')
    plt.xlabel('Distance (m)')
    plt.ylabel('Intensity')
    plt.title('Intensity vs Distance')
    plt.grid(True)
    plt.legend()
    plt.savefig(f'data/image/attenuation/Intensity_vs_Distance_{file_name}.png', dpi=600)
    plt.show()

def process_bin_file(bin_file_path, alpha=0.01):
    points = load_velodyne_points(bin_file_path)
    
    # 计算衰减后的反射强度
    calculated_intensities = calculate_attenuation(points, alpha)
    
    # 可视化
    plot_attenuation(points, calculated_intensities, os.path.basename(bin_file_path))

    # 比较原始强度和衰减后的强度
    print(f"Original Intensity (first point): {points[0, 3]}")
    print(f"Calculated Intensity (first point): {calculated_intensities[0]}")
    
    # 计算衰减后的反射强度的平均值
    mean_intensity = np.mean(calculated_intensities)
    print(f"Mean Calculated Intensity: {mean_intensity}")
    
    return calculated_intensities, mean_intensity

if __name__ == '__main__':
    final_mean_intensity = 0

    bin_file_folder = 'data/lidar_points'
    for bin_file in os.listdir(bin_file_folder):
        if bin_file.endswith('.bin'):
            attenuated_intensities, mean_intensity = process_bin_file(os.path.join(bin_file_folder, bin_file), alpha=0.01)
            final_mean_intensity += mean_intensity
            print(f"Processed file: {bin_file}, Mean Calculated Intensity: {mean_intensity}")
    
    print(final_mean_intensity/len(os.listdir(bin_file_folder)))
