# Coveyor Sorter

## Requirements

- ROS 2 Jazzy Jalisco
*(Developed using `osrf/ros:jazzy-desktop-full` Docker image)*
- `ros_gz_sim` package
- `vision_opencv` package

## Installation

### Building from source

```sh
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Usage

### Launch

```sh
ros2 launch conveyor_sorter init.launch.py
```

Spawn multiple random props and start the conveyor:
```sh
ros2 run conveyor_sorter random
```
```
```

## Structure

### Services

- `/conveyor/start`  
  Starts the conveyor. May go backwards by setting `forward` to false. May go with different speed by setting `speed`.
- `/pusher/push`  
  Performs a single push by the pusher.
- `/props/spawn`  
  Spawns a prop on the conveyor. May spawn random from the pool by setting `random` to true, or may be chosen manually by setting `random` to false and varying `target` (if true, it will be sorted out).
