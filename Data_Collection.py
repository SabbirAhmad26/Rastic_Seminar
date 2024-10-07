#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from pylimo import limo
from ackermann_msgs.msg import AckermannDrive
import time
import numpy as np
import cv2
import json
import os
import threading
from cav_project.msg import limo_info, ControlInfo
from LidarX2 import LidarX2
import datetime

class LimoInfoPublisher:
    def __init__(self, ID):
        self.ID = ID
        rospy.init_node('limo_info_publisher_'+self.ID)

        # Initialize ROS publishers and subscribers
        self.info_pub_cav1 = rospy.Publisher('/limo_info_'+self.ID, limo_info, queue_size=10)
        self.control_info_sub_cav1 = rospy.Subscriber("control_info_"+self.ID, ControlInfo, self.control_info_callback)

        self.limo_data_cav1 = limo_info()
        self.control_info_cav1 = ControlInfo()
        self.rate = rospy.Rate(10)

        # Initialize Limo and LiDAR
        self.limo = limo.LIMO()
        self.limo.EnableCommand()
        self.lidar = LidarX2('/dev/ttyUSB0')

        # Initialize camera for data collection
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Create directories for storing data
        dir_date = datetime.datetime.now().strftime("%m-%d-%Y")
        self.date_exp = os.path.join(dir_date, "test_limo")
        if not os.path.exists(self.date_exp):
            os.makedirs(self.date_exp)
            self.full_path = os.path.join(self.date_exp, "run_001")
        else:
            directories = [d for d in os.listdir(self.date_exp) if os.path.isdir(os.path.join(self.date_exp, d))]
            run_number = len(directories) + 1
            self.full_path = os.path.join(self.date_exp, f"run_{run_number:03d}")

        os.makedirs(self.full_path, exist_ok=True)
        self.data_dir_rgb = os.path.join(self.full_path, "rgb")
        self.data_dir_label = os.path.join(self.full_path, "label_data")
        os.makedirs(self.data_dir_rgb, exist_ok=True)
        os.makedirs(self.data_dir_label, exist_ok=True)

        self.frame_count = 0
        self.collect_data = True

    def control_info_callback(self, data):
        self.control_info_cav1 = data

    def publish_info(self):
        self.info_pub_cav1.publish(self.limo_data_cav1)

    def save_data(self, image_filename, frame, sensor_data_filename, json_str):
        # Save image and sensor data
        cv2.imwrite(image_filename, frame)
        with open(sensor_data_filename, 'w') as file:
            file.write(json_str)

    def run(self):
        steering_angle = np.zeros(250)
        imu_yaw = np.zeros(250)
        iter = 0

        while not rospy.is_shutdown():
            frame_start_time = time.time()  # Start time for FPS

            if self.control_info_cav1.desired_velocity is not None:
                # Set motion command
                self.limo.SetMotionCommand(linear_vel=self.control_info_cav1.desired_velocity,
                                           steering_angle=self.control_info_cav1.steering_angle)

                # Print the velocity and steering commands
                print("Limo velocity command:", self.control_info_cav1.desired_velocity,
                      "Limo steering:", self.control_info_cav1.steering_angle)

                # Get actual velocity from the robot
                try:
                    actual_velocity = self.limo.GetLinearVelocity()
                except AttributeError:
                    actual_velocity = 0.0

                # Print the actual velocity
                print("actual_velocity:", actual_velocity)

                self.limo_data_cav1.vel.data = actual_velocity
                self.publish_info()

                # Capture sensor and image data
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    rospy.logerr("Failed to capture image from camera")
                    break

                # Get LiDAR measurements
                measures = self.lidar.getMeasures()

                # Package all sensor data, including LiDAR data
                sensor_data = {
                    'EpochTime': time.time(),
                    'DateTime': datetime.datetime.now().strftime("%m-%d-%Y_%H-%M-%S"),
                    'LinearVelocity': actual_velocity,
                    'AngularVelocity': self.limo.GetAngularVelocity() if hasattr(self.limo, 'GetAngularVelocity') else None,
                    'SteeringAngle': self.limo.GetSteeringAngle() if hasattr(self.limo, 'GetSteeringAngle') else None,
                    'LateralVelocity': self.limo.GetLateralVelocity() if hasattr(self.limo, 'GetLateralVelocity') else None,
                    'ControlMode': self.limo.GetControlMode() if hasattr(self.limo, 'GetControlMode') else None,
                    'BatteryVoltage': self.limo.GetBatteryVoltage() if hasattr(self.limo, 'GetBatteryVoltage') else None,
                    'ErrorCode': self.limo.GetErrorCode() if hasattr(self.limo, 'GetErrorCode') else None,
                    'RightWheelOdom': self.limo.GetRightWheelOdom() if hasattr(self.limo, 'GetRightWheelOdom') else None,
                    'LeftWheelOdom': self.limo.GetLeftWheelOdom() if hasattr(self.limo, 'GetLeftWheelOdom') else None,
                    'Acceleration': self.limo.GetIMUAccelData() if hasattr(self.limo, 'GetIMUAccelData') else None,
                    'Gyro': self.limo.GetIMUGyroData() if hasattr(self.limo, 'GetIMUGyroData') else None,
                    'Yaw': self.limo.GetIMUYawData() if hasattr(self.limo, 'GetIMUYawData') else None,
                    'Pitch': self.limo.GetIMUPitchData() if hasattr(self.limo, 'GetIMUPitchData') else None,
                    'Roll': self.limo.GetIMURollData() if hasattr(self.limo, 'GetIMURollData') else None,
                    'Command': {
                        'Throttle': self.control_info_cav1.desired_velocity,
                        'Steer': self.control_info_cav1.steering_angle
                    },
                    'LiDAR': [{'Angle': measure.angle, 'Distance': measure.distance} for measure in measures]  # Split LiDAR data
                }

                # Convert sensor data to JSON string
                json_str = json.dumps(sensor_data, indent=4)

                # Save data
                if self.collect_data:
                    image_filename = f"{self.data_dir_rgb}/{self.frame_count:09d}.jpg"
                    sensor_data_filename = f"{self.data_dir_label}/{self.frame_count:09d}.json"
                    save_thread = threading.Thread(target=self.save_data, args=(image_filename, frame, sensor_data_filename, json_str))
                    save_thread.start()

                self.frame_count += 1

                # FPS calculation and logging
                frame_end_time = time.time()
                elapsed_time = frame_end_time - frame_start_time
                fps = 1 / elapsed_time if elapsed_time > 0 else 0
                print(f"Time: {elapsed_time:.2f}")
                print(f"FPS: {fps:.2f}")

                iter += 1
                time.sleep(0.1)

                self.rate.sleep()
            else:
                rospy.logwarn("No control info received, skipping iteration.")
                if iter > 10:
                    break

        self.cap.release()

if __name__ == '__main__':
    publisher = LimoInfoPublisher("limo793")
    publisher.run()
