## Installation
Clone this repository
```
git clone https://github.com/pollen-robotics/zuuu_follow_me.git
```

Clone and install **our custom version of PyVESC**:
```
git clone https://github.com/pollen-robotics/PyVESC.git
pip3 install -e PyVESC/
```

Install:
```
sudo apt install ros-foxy-tf-transformations
pip3 install transforms3d
```

## Usage
### Running the HAL
For all ROS based use cases, the zuuu_hal must be started with :
```
ros2 run zuuu_follow_me hal
```

### Sending speed commands
Once the zuuu_hal is started, you can take control of the mobile base with:
1) A controller
```
ros2 run zuuu_follow_me teleop_joy
```
2) A keyboard
```
ros2 run zuuu_follow_me teleop_keyboard
```
3) With code, sending speed commands on the 'cmd_vel' topic.


### Testing the odometry
You can run a visual test using RViz (setting a high value to the LIDAR decay time is a visual trick to see the integral of the errors of the odometry):
```
ros2 launch zuuu_description rviz_bringup.launch.py
```

### Follow me demo
Old code that requires a controller to be connected. Allows for controller control or automatic "follow me" behaviour.
This demo does not require the HAL to be started, just run: 
```
ros2 launch zuuu_follow_me follow_me_launch.py
```
