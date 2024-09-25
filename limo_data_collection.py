import os
import cv2
import json
import pygame
import datetime
import time
import argparse
import numpy as np
from pylimo import limo


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

# Inits
limo=limo.LIMO()
limo.EnableCommand()
pygame.init()
pygame.joystick.init()

# Make sure at least one joystick is connected
if pygame.joystick.get_count() == 0:
    print("No joystick detected")
    exit(1)

# Get the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Joystick Name: {joystick.get_name()}")

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
collect_data = False


while True:
    if args.use_imutils:
        frame = cap.read()
    else:
        _, frame = cap.read()

    if frame is None:
        break

    # Get the latest events
    pygame.event.get()

    # Throttle
    throttle = -joystick.get_axis(1) / args.divide_throttle
    throttle = np.round(throttle, 4)

    # Steering
    steer = -joystick.get_axis(3) / args.divide_steer
    steer = np.round(steer, 4) 


    # Collect Data
    if joystick.get_axis(5) > 0.5:
        collect_data = True
    if joystick.get_axis(2) > 0.5:
        collect_data = False

    # Command
    if joystick.get_button(2):
        command = "Left"
    elif joystick.get_button(3):
        command = "Straight"
    elif joystick.get_button(1):
        command = "Right"
    # elif joystick.get_button(0):
    #     command = "LaneFollow"
    # elif joystick.get_button(5):
    #     command = "LaneChaneRight"
    # elif joystick.get_button(4):
    #     command = "LaneChangeLeft"


    # Save the controls
    image_filename = f"{data_dir_rgb}/{frame_count:09d}.jpg"
    try:
        left_wheel_odom = limo.GetLeftWheelOdeom()
    except:
        left_wheel_odom = limo.GetLeftWheelOdom()

    sensor_data = {
        'EpochTime': time.time(),
        'DateTime': datetime.datetime.now().strftime("%m-%d-%Y_%H-%M-%S"),
        'LinearVelocity': limo.GetLinearVelocity(),
        'AngularVelocity': limo.GetAngularVelocity(),
        'SteeringAngle': limo.GetSteeringAngle(),
        'LateralVelocity': limo.GetLateralVelocity() ,
        'ControlMode': limo.GetControlMode(),
        'BatteryVoltage': limo.GetBatteryVoltage(),
        'ErrorCode': limo.GetErrorCode(),
        'RightWheelOdom':limo.GetRightWheelOdom() ,
        'LeftWheelOdom': left_wheel_odom,
        'Acceleration': limo.GetIMUAccelData(),
        'Gyro': limo.GetIMUGyroData(),
        'Yaw': limo.GetIMUYawData(),
        'Pitch': limo.GetIMUPichData(),
        'Roll':limo.GetIMURollData(),
        'Command': command,
        'Throttle': throttle,
        'Steer': steer,
        'Location': []
    }

    json_str = json.dumps(sensor_data, indent=4)
    if collect_data:
        with open(f"{data_dir_label}/{frame_count:09d}.json", 'w') as file:
            file.write(json_str)

        cv2.imwrite(image_filename, frame)
        frame_count += 1
        print(f"C: {command} | T: {throttle:.2f}, S: {steer:.2f}")
    else:
        print("Not collecting data")

    # Set the controls
    limo.SetMotionCommand(linear_vel=throttle, steering_angle=steer)

    
    cv2.imshow("Frame", frame)
    

    # Wait for a short while to avoid excessive polling
    pygame.time.wait(10)
    key = cv2.waitKey(1)

    if key == ord("q"):
        break