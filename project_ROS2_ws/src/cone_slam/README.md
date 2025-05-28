# Cone SLAM Node (ROS 2)

This ROS 2 node performs a simple SLAM algorithm using detected cones from a stereo camera.

## Subscribed Topics
- `/cone_detections_3D` — `std_msgs/msg/String`  
  Cone detections in format: `"B;100;0;2500|O;-150;0;2700"`

## Features
- EKF-based motion tracking
- Landmark map creation using cones
- Real-time top-down 2D visualization using matplotlib

## Usage of it

Run SLAM node:
```bash
ros2 run cone_slam cone_slam
