# ros2dss
A tool for verification of a distributed system modeled by a modular Petri net based on ROS2. Each module (robot) builds an improved version of its state space that allows to check its properties without exploring the state spaces of other modules. 


## **Installation**

Clone the repository into your ROS2 workspace and build:

```bash
cd ~/ros2_ws/src
git clone https://github.com/chihebabid/ros2dss.git ros2dss_project
cd ~/ros2_ws
colcon build --packages-select ros2dss_project
```

## **Usage**
To run the tool, you can use the following command:

```bash
ros2 launch ros2dss_project launch.py file1:=<model_file> enable_reduction:=true
```

Parameters:

- file1, file2, ...,: Name(s) of Petri net files to verify.

- `enable_reduction`: true to enable metastate reduction (-r flag), false otherwise.
