# RX200 Teleoperation with Haptic Feedback

A teleoperation system enabling intuitive control of the interbotix RX200 robotic arm or any interbotix arm using the Geomagic Touch haptic device, providing real-time force feedback for enhanced manipulation precision.

## Setup

* Interbotix RX200: A 5-DOF robotic manipulator arm with precise servo motors and end-effector control
* Geomagic Touch: A 6-DOF haptic input device providing force feedback capabilities for immersive teleoperation
* Zenoh v1.6.2 : A DDS and DDS-bridge for ROS2 communication system supporting both localhost and server-based teleoperation deployments.
* ROS2 Humble
* Ubuntu 22.04.5 LTS

## Installation

```bash
sudo apt install curl
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
./xsarm_amd64_install.sh -d humble
```