import os
import cv2
import json
import datetime
import time
import argparse
import numpy as np
from pylimo import limo
from LidarX2 import LidarX2
from time import sleep
import threading

parser = argparse.ArgumentParser()
parser.add_argument('--exp', type=str, default="test_limo", help="Experiment name")
parser.add_argument('--divide_throttle', type=float, default=2.0, help='Divide the throttle by this amount')
parser.add_argument('--divide_steer', type=float, default=2.0, help='Divide the steering by this amount')
parser.add_argument('--use_imutils', action='store_true', help='Use imutils to get the frame')

args = parser.parse_args()

if args.use_imutils:
    from imutils.video import VideoStream
    cap = VideoStream(src=0).start()
else:
    cap = cv2.VideoCapture(0)
    width = 640
    height = 480
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)  # Set width to 1280 pixels
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)  # Set height to 720 pixels

# Initialize LIMO robot
limo = limo.LIMO()
limo.EnableCommand()

# Initialize LiDAR
lidar = LidarX2('/dev/ttyUSB0')  # Change to appropriate serial port

# Create directories for data storage
dir_date = datetime.datetime.now().strftime("%m-%d-%Y")
date_exp = os.path.join(dir_date, args.exp)

if not os.path.exists(date_exp):
    os.makedirs(date_exp)
    full_path = os.path.join(date_exp, "run_001")
else:
    directories = [d for d in os.listdir(date_exp) if os.path.isdir(os.path.join(date_exp, d))]
    run_number = len(directories) + 1
    full_path = os.path.join(date_exp, f"run_{run_number:03d}")

os.makedirs(full_path, exist_ok=True)

data_dir_rgb = os.path.join(full_path, "rgb")
data_dir_label = os.path.join(full_path, "label_data")
os.makedirs(data_dir_rgb, exist_ok=True)
os.makedirs(data_dir_label, exist_ok=True)

frame_count = 0
command = "Straight"
collect_data = True  # Set to True to collect data

# FPS calculation variables
fps = 0
start_time = time.time()

# Open LiDAR connection
if lidar.open():
    print("LIDAR connection opened successfully.")
else:
    print("Failed to open LIDAR connection.")
    exit()

# Function to save image and sensor data in a separate thread
def save_data(image_filename, frame, sensor_data_filename, json_str):
    # Save the frame as an image
    cv2.imwrite(image_filename, frame)

    # Write sensor data to JSON file
    with open(sensor_data_filename, 'w') as file:
        file.write(json_str)

# Main loop
try:
    while True:
        frame_start_time = time.time()  # Start time for this frame

        # Get LiDAR measures
        measures = lidar.getMeasures()

        # Capture video frame
        if args.use_imutils:
            frame = cap.read()
        else:
            _, frame = cap.read()

        if frame is None:
            break

        throttle = 0
        steer = 0

        # Save the controls
        image_filename = f"{data_dir_rgb}/{frame_count:09d}.jpg"
        sensor_data_filename = f"{data_dir_label}/{frame_count:09d}.json"
        try:
            left_wheel_odom = limo.GetLeftWheelOdeom()
        except:
            left_wheel_odom = limo.GetLeftWheelOdom()

        # Package all sensor data, including LiDAR data
        sensor_data = {
            'EpochTime': time.time(),
            'DateTime': datetime.datetime.now().strftime("%m-%d-%Y_%H-%M-%S"),
            'LinearVelocity': limo.GetLinearVelocity(),
            'AngularVelocity': limo.GetAngularVelocity(),
            'SteeringAngle': limo.GetSteeringAngle(),
            'LateralVelocity': limo.GetLateralVelocity(),
            'ControlMode': limo.GetControlMode(),
            'BatteryVoltage': limo.GetBatteryVoltage(),
            'ErrorCode': limo.GetErrorCode(),
            'RightWheelOdom': limo.GetRightWheelOdom(),
            'LeftWheelOdom': left_wheel_odom,
            'Acceleration': limo.GetIMUAccelData(),
            'Gyro': limo.GetIMUGyroData(),
            'Yaw': limo.GetIMUYawData(),
            'Pitch': limo.GetIMUPichData(),
            'Roll': limo.GetIMURollData(),
            'Command': command,
            'Throttle': throttle,
            'Steer': steer,
            'Location': [],
            'LiDAR': [{'Angle': measure.angle, 'Distance': measure.distance} for measure in measures]  # Split LiDAR data
        }

        json_str = json.dumps(sensor_data, indent=4)

        # Start a new thread for saving the data if collecting
        if collect_data:
            save_thread = threading.Thread(target=save_data, args=(image_filename, frame, sensor_data_filename, json_str))
            save_thread.start()  # Start saving data in a new thread

        frame_count += 1

        # Set motion commands for the robot
        limo.SetMotionCommand(linear_vel=throttle, steering_angle=steer)

        # Calculate FPS
        frame_end_time = time.time()
        elapsed_time = frame_end_time - frame_start_time
        fps = 1 / elapsed_time
        print(f"Time: {elapsed_time:.2f}")
        print(f"FPS: {fps:.2f}")

        # Exit loop if 'q' is pressed
        key = cv2.waitKey(1)
        if key == ord("q"):
            break

# Safely close connections on exit
finally:
    lidar.close()
    print("LIDAR connection closed.")

