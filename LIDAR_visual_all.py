from LIDAR_visual import plot_LIDAR
import os

data_dir = ''

for file_name in os.listdir(data_dir):
    plot_LIDAR(data_dir, file_name)