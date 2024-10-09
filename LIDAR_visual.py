import os
import json
import numpy as np
import matplotlib.pyplot as plt

# Convert Polar to Cartesian coordinates
def polar_to_cartesian(angle, distance):
    angle_rad = np.radians(angle)
    x = distance * np.cos(angle_rad)
    y = distance * np.sin(angle_rad)
    return x, y

# Plot for one frame
def plot_LIDAR(data_dir, file_name):

    # Store all the points for current frame 
    all_points = []
    x_coords = []
    y_coords = []
    the_path = os.path.join(data_dir, file_name)

    with open(the_path, 'r') as f:
        data = json.load(f)
       # data = {"LiDAR": [{"Angle": 225 - i, "Distance": 20} for i in range(270)]} # for testing purpose
        lidar_data = data['LiDAR']

        for measure in lidar_data:
            angle = measure['Angle']
            distance = measure['Distance']

            x, y = polar_to_cartesian(angle, distance)
            all_points.append((x, y))
            x_coords.append(x)
            y_coords.append(y)

    # Plot the LiDAR points in 2D (not related to rviz)
    fig = plt.figure(figsize=(10, 10))
    plt.scatter(x_coords, y_coords, s=1, c='blue')
    plt.title('2D Visualization')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.axis('equal') 
    plt.show()
    fig.savefig(the_path[:-5]+'.png')

    return all_points

# JSON files path
data_dir = ""

# All JSON files
#for file_name in os.listdir(data_dir):
#    plot_LIDAR(data_dir, file_name)
    
plot_LIDAR("UROP Fall 2024","test.json")