# Turtlebot PID Controller in ROS2

Please find the instructions below to run the controller.

```bash
# Create a new workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Clone the repository into src
git clone https://github.com/SabbirAhmad26/Rastic_Seminar/tree/main/Seminar%204%20-%20ROS2.git

Build the package
```
cd ~/ros2_ws
colcon build
```

Source ROS2 
```
source /opt/ros/foxy/setup.bash
```

Source the workspace
```
source install/local_setup.bash
```

Launch the ROS2 node
```
ros2 run turtleController controller
```
