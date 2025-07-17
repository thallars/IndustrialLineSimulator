# IndustrialLineSimulator
## Requirements
- ROS 2 Jazzy Jalisco 
*(Developed using `osrf/ros:jazzy-desktop-full` Docker image)*
- `ros_gz_sim` package
## Usage
Run:
```
source /opt/ros/jazzy/setup.bash
gz sim conveyor_sorter/worlds/conveyor_world.sdf
```
In another terminal:
```
ros2 run ros_gz_sim create -file conveyor_sorter/models/conveyor/model.sdf
ros2 run ros_gz_sim create -file conveyor_sorter/models/pusher/model.sdf
```
To send a command to the pusher and belt:
```
gz topic -t "/rod_vel" -m gz.msgs.Double -p "data: 10.0"
gz topic -t "/belt_vel" -m gz.msgs.Double -p "data: 0.0"
```
