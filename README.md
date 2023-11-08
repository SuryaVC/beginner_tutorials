[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# Simple ROS2 Publisher/Subscriber

Author: Surya Chappidi <br>
UID: 119398166

## Overview
This repository provides a simple ROS2 Publisher/Subscriber example.

## Assumptions
Before you proceed with building and running this project, please ensure the following:

- ROS2 is already set up locally on your system.
- You have a ROS2 workspace named `ros2_ws` in your home directory.

## Build
### First-Time Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/SuryaVC/beginner_tutorials
cd ..
colcon build --packages-select cpp_pubsub
. install/setup.bash
```
## Run

```bash
ros2 run cpp_pubsub talker
```
Then, open a new terminal and run the subscriber:
```bash
. install/setup.bash
ros2 run cpp_pubsub listener
```

To quit the talker and listener, press `Ctrl+C` in their respective terminals.

## Checks
### cpplint:

```bash
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" )
```

### cppcheck:
```bash
cppcheck --enable=all --std=c++11 --check-config --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" )
```

## Results:
Cpplint and Cppcheck output are in results directory.


## Dependencies:
rclcpp <br>
std_msgs
