#!/usr/bin/env python3

import os
import cv2
import json
import datetime
import time
import argparse
import numpy as np
import pickle
import threading
from pylimo import limo
from ackermann_msgs.msg import AckermannDrive
from LidarX2 import LidarX2
import rospy

class listener:
    def __init__(self):
        self.data = None
        self.xprev = 0
        self.yprev = 0

    def callback(self, data):
        self.data = data

    def listener(self):
        rospy.init_node("vs_listener", anonymous=True)
        rospy.Subscriber("vel_steer_limo793", AckermannDrive, self.callback)

class LimoDataCollection:
    def __init__(self, exp_name, lidar_port, divide_throttle=2.0, divide_steer=2.0, use_imutils=False):
        self.exp_name = exp_name
        self.divide_throttle = divide_throttle
        self.divide_steer = divide_steer
        self.use_imutils = use_imutils
        self.listener_ins = listener()

        # Initialize LIMO robot
        self.limo = limo.LIMO()
        self.limo.EnableCommand()

        # Initialize LiDAR
        self.lidar = LidarX2(lidar_port)
        if self.lidar.open():
            print("LIDAR connection opened successfully.")
        else:
            print("Failed to open LIDAR connection.")
            exit()

        # Prepare for video capture
        if self.use_imutils:
            from imutils.video import VideoStream
            self.cap = VideoStream(src=0).start()
        else:
            self.cap = cv2.VideoCapture(0)
            width = 640
            height = 480
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        # Set up directories
        dir_date = datetime.datetime.now().strftime("%m-%d-%Y")
        date_exp = os.path.join(dir_date, self.exp_name)
        if not os.path.exists(date_exp):
            os.makedirs(date_exp)
            self.full_path = os.path.join(date_exp, "run_001")
        else:
            directories = [d for d in os.listdir(date_exp) if os.path.isdir(os.path.join(date_exp, d))]
            run_number = len(directories) + 1
            self.full_path = os.path.join(date_exp, f"run_{run_number:03d}")

        os.makedirs(self.full_path, exist_ok=True)
        self.data_dir_rgb = os.path.join(self.full_path, "rgb")
        self.data_dir_label = os.path.join(self.full_path, "label_data")
        os.makedirs(self.data_dir_rgb, exist_ok=True)
        os.makedirs(self.data_dir_label, exist_ok=True)

        # Motion and data collection
        self.frame_count = 0
        self.command = "Straight"
        self.collect_data = True
        self.fps = 0
        self.start_time = time.time()

    def save_data(self, image_filename, frame, sensor_data_filename, json_str):
        # Save the frame as an image
        cv2.imwrite(image_filename, frame)

        # Write sensor data to JSON file
        with open(sensor_data_filename, 'w') as file:
            file.write(json_str)

    def run(self):
        iter = 0
        linear_vel = 0  # Initialize with default value
        steering_angle = 0  # Initialize with default value
        while True:
            self.listener_ins.listener()

            frame_start_time = time.time()

            # Get LiDAR measures
            measures = self.lidar.getMeasures()

            # Capture video frame
            if self.use_imutils:
                frame = self.cap.read()
            else:
                _, frame = self.cap.read()

            if frame is None:
                break

            if self.listener_ins.data is not None:
                # Set motion commands from ROS topic
                linear_vel = self.listener_ins.data.speed
                steering_angle = self.listener_ins.data.steering_angle
                self.limo.SetMotionCommand(linear_vel=linear_vel, steering_angle=steering_angle)

                print(f"Limo velocity command: {linear_vel}, Limo steering: {steering_angle}")

            # Collect sensor data
            try:
                left_wheel_odom = self.limo.GetLeftWheelOdom()
            except:
                left_wheel_odom = None

            sensor_data = {
                'EpochTime': time.time(),
                'DateTime': datetime.datetime.now().strftime("%m-%d-%Y_%H-%M-%S"),
                'LinearVelocity': self.limo.GetLinearVelocity(),
                'AngularVelocity': self.limo.GetAngularVelocity(),
                'SteeringAngle': self.limo.GetSteeringAngle(),
                'LateralVelocity': self.limo.GetLateralVelocity(),
                'ControlMode': self.limo.GetControlMode(),
                'BatteryVoltage': self.limo.GetBatteryVoltage(),
                'ErrorCode': self.limo.GetErrorCode(),
                'RightWheelOdom': self.limo.GetRightWheelOdom(),
                'LeftWheelOdom': left_wheel_odom,
                'Acceleration': self.limo.GetIMUAccelData(),
                'Gyro': self.limo.GetIMUGyroData(),
                'Yaw': self.limo.GetIMUYawData(),
                'Pitch': self.limo.GetIMUPitchData() if hasattr(self.limo, 'GetIMUPitchData') else None,  # Check for Pitch data
                'Roll': self.limo.GetIMURollData() if hasattr(self.limo, 'GetIMURollData') else None,
                'Command': {
                    'Throttle': linear_vel,
                    'Steer': steering_angle
                },
                'LiDAR': [{'Angle': measure.angle, 'Distance': measure.distance} for measure in measures]
            }

            json_str = json.dumps(sensor_data, indent=4)
            image_filename = f"{self.data_dir_rgb}/{self.frame_count:09d}.jpg"
            sensor_data_filename = f"{self.data_dir_label}/{self.frame_count:09d}.json"

            if self.collect_data:
                save_thread = threading.Thread(target=self.save_data, args=(image_filename, frame, sensor_data_filename, json_str))
                save_thread.start()

            self.frame_count += 1
            iter += 1

            frame_end_time = time.time()
            elapsed_time = frame_end_time - frame_start_time
            self.fps = 1 / elapsed_time
            print(f"Time: {elapsed_time:.2f} seconds, FPS: {self.fps:.2f}")

            key = cv2.waitKey(1)
            if key == ord("q"):
                break

        self.lidar.close()
        print("LIDAR connection closed.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--exp', type=str, default="test_limo", help="Experiment name")
    parser.add_argument('--divide_throttle', type=float, default=2.0, help='Divide the throttle by this amount')
    parser.add_argument('--divide_steer', type=float, default=2.0, help='Divide the steering by this amount')
    parser.add_argument('--use_imutils', action='store_true', help='Use imutils to get the frame')

    args = parser.parse_args()

    limo_data_collection = LimoDataCollection(
        exp_name=args.exp,
        lidar_port='/dev/ttyUSB0',
        divide_throttle=args.divide_throttle,
        divide_steer=args.divide_steer,
        use_imutils=args.use_imutils
    )

    limo_data_collection.run()
