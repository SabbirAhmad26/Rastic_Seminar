
# Imitation Learning for Autonomous Driving in Lab Testbed

This project implements imitation learning for autonomous driving, specifically for navigating an intersection and merging onto a highway in a controlled lab testbed environment. The goal is to train a vehicle to mimic expert driving behavior using demonstrations and apply the learned policy to real-time decision-making in traffic scenarios.

## Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [Installation](#installation)
- [Usage](#usage)
- [Testbed Setup](#testbed-setup)
- [Imitation Learning Approach](#imitation-learning-approach)
  - [Data Collection](#data-collection)
  - [Model Training](#model-training)
  - [Policy Execution](#policy-execution)
- [Evaluation](#evaluation)
- [Contributing](#contributing)
- [License](#license)

## Overview

Imitation learning is a technique used to train autonomous systems by learning from demonstrations. In this project, we focus on autonomous driving in a lab testbed containing two key traffic scenarios:
- Navigating through an intersection.
- Merging onto a highway.

The goal is to develop a model that can imitate human driving behavior in these scenarios, effectively handling traffic rules, vehicle interactions, and lane changes.

### Key Components:
- **Intersection Navigation:** Handling right-of-way, stopping, and acceleration through intersections.
- **Highway Merging:** Smooth lane changes and safe merging into traffic.

## Project Structure

```bash
├── data/                   # Collected expert demonstration data
├── src/                    # Source code for training and inference
│   ├── models/             # Neural network architectures for imitation learning
│   ├── train.py            # Script to train the imitation learning model
│   ├── inference.py        # Script to test the trained policy
│   ├── utils.py            # Utility functions for data processing
├── testbed/                # Testbed environment setup and configuration files
├── README.md               # Project documentation
└── requirements.txt        # Python dependencies
```

## Installation

### Prerequisites
- Python 3.7 or higher
- TensorFlow/PyTorch (depending on your choice of framework)
- OpenCV (for visualization and data processing)
- ROS (Robot Operating System) for real-time control in the testbed

Clone the repository and install the required dependencies:

```bash
git clone https://github.com/yourusername/autonomous-driving-imitation-learning.git
cd autonomous-driving-imitation-learning
pip install -r requirements.txt
```

## Usage

### 1. Collect Expert Demonstration Data
Before training, gather expert driving demonstrations by controlling the vehicle manually through the testbed and recording sensor data (e.g., LIDAR, camera, IMU) along with control commands (steering, throttle, braking).

```bash
python src/data_collection.py
```

### 2. Train the Imitation Learning Model
Once the data is collected, train the imitation learning model using the following command:

```bash
python src/train.py --data data/ --epochs 50
```

### 3. Test the Learned Policy
After training, test the policy on the lab testbed:

```bash
python src/inference.py --model saved_models/model.pth
```

## Testbed Setup

The lab testbed simulates realistic traffic scenarios with an intersection and a highway merging point. The testbed can be set up using small robotic cars (such as F1Tenth or DonkeyCar) equipped with sensors (cameras, LIDAR, GPS). ROS is used for real-time control, data collection, and simulation of vehicle dynamics.

**Key Features:**
- Intersection with traffic signals and other vehicles.
- Highway stretch for merging simulations.

Ensure the testbed environment is properly calibrated for sensor data and vehicle control before running experiments.

## Imitation Learning Approach

### Data Collection

The first step is to collect data from expert drivers. This involves driving a vehicle through the testbed and capturing sensor readings along with control inputs (e.g., steering, acceleration).

### Model Training

The imitation learning model is trained using supervised learning, where the input is sensor data and the output is the corresponding control command. We experiment with different neural network architectures such as:
- **Convolutional Neural Networks (CNNs)** for image-based input.
- **Recurrent Neural Networks (RNNs)** for time-series input.

### Policy Execution

Once trained, the model is deployed on the vehicle in the testbed. The vehicle uses the learned policy to make real-time driving decisions based on sensor input. We implement:
- **Behavior Cloning (BC):** The model directly maps sensor inputs to driving commands.
- **DAgger (Dataset Aggregation):** To improve performance, we implement an iterative approach where the model is fine-tuned with corrections from expert feedback.

## Evaluation

We evaluate the model's performance using the following metrics:
- **Success Rate:** Percentage of successful intersection navigation and highway merging.
- **Collision Rate:** Number of collisions during test runs.
- **Smoothness:** Evaluated based on control inputs (steering and acceleration).

## Contributing

Contributions are welcome! Please follow the standard GitHub workflow:
1. Fork the repository.
2. Create a feature branch.
3. Commit your changes.
4. Open a pull request.

Feel free to submit issues or suggestions for improvements.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.
