# RX200 Teleoperation with Haptic Feedback

A teleoperation system enabling intuitive control of the interbotix RX200 robotic arm or any interbotix arm using the Geomagic Touch haptic device, providing real-time force feedback for enhanced manipulation precision.

## Setup

* Interbotix RX200: A 5-DOF robotic manipulator arm with precise servo motors and end-effector control
* Geomagic Touch: A 6-DOF haptic input device providing force feedback capabilities for immersive teleoperation
* Zenoh v1.6.2 : Zenoh is a fast communication middleware used to share data between applications, robots, and networks. It can work peer-to-peer or through a Zenoh router.
* ROS2 Humble
* Ubuntu 22.04.5 LTS

## Installation

```bash
git clone https://github.com/KaranK369/rx200_Teleoperation_with_Haptic_Feedback.git
```
* Install the Drivers for geomagic touch

* zenoh for teleoperation

```bash
curl -L https://download.eclipse.org/zenoh/debian-repo/zenoh-public-key | sudo gpg --dearmor --yes -o /etc/apt/keyrings/zenoh-public-key.gpg
echo "deb [signed-by=/etc/apt/keyrings/zenoh-public-key.gpg] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee /etc/apt/sources.list.d/zenoh.list

sudo apt update
sudo apt install zenohd                 # for the Zenoh router daemon
sudo apt install zenoh-bridge-ros2dds   # alternative standalone bridge executable
sudo apt install zenoh-plugin-ros2dds   # for ROS2-DDS bridge plugin (if needed)
```

verify

```bash
zenohd --version
zenoh-bridge-ros2dds --version
```

# setup zenoh

To test Zenoh with ROS 2, run:

* Make sure that ROS_DOMAIN_ID is same for both of the divices
* you can set it in you bashrc file in both of the divices
```bash
export ROS_DOMAIN_ID=7 # for example
```
verify:
```bash
echo $ROS_DOMAIN_ID
```

1. Zenoh Router (zenohd) – the central node that forwards Zenoh traffic.
Start it first:
```bash
zenohd -l tcp/0.0.0.0:7447
```
* 7477 is default port for zenoh

2. zenoh–ROS2 Bridge – connects ROS 2 topics to the Zenoh network.
Run it after the router:
```bash
zenoh-bridge-ros2dds --connect tcp/<router-ip>:7447
```
* Check router's divice ip:
```bash
ifconfig
```

# Uses

Build and source the scr files of interbotix_ws and Geomagic_Touch_ROS2 individually 

```bash
colcon build
source install/setup.bash
```

# Simulation
'''bash
ros2 launch interbotix_xsarm_sim xsarm_gz_classic.launch.py robot_model:=rx200
ros2 launch omni_common omni.launch.py 
'''
nevigate to interbotix_ws/src/controller and run
'''bash
python3 arm_control_sim.py
python3 effort_controller.py
```
nevigate to geomagic_touch_ros2/src/omni_common/src and run
```bash
python3 ff.py
```

# Real arm
'''bash
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=rx200 
ros2 launch omni_common omni.launch.py 
'''
nevigate to geomagic_touch_ros2/src/omni_common/src and run
```bash
python3 ff.py
```