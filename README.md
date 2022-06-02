## Installation
Clone and install our custom version of PyVESC:

https://github.com/pollen-robotics/PyVESC

```
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

### Setting the drive mode
Can be tested with CLI:
```
ros2 service call /SetZuuuMode zuuu_interfaces/srv/SetZuuuMode "{mode: BRAKE}" 
ros2 service call /SetZuuuMode zuuu_interfaces/srv/SetZuuuMode "{mode: FREE_WHEEL}" 
ros2 service call /SetZuuuMode zuuu_interfaces/srv/SetZuuuMode "{mode: DRIVE}" 
```

### Testing the odometry
You can run a visual test using RViz (setting a high value to the LIDAR decay time is a visual trick to see the integral of the errors of the odometry):
```
ros2 launch zuuu_description rviz_bringup.launch.py
```

Getting the odometry in CLI:
```
ros2 service call /GetOdometry zuuu_interfaces/srv/GetOdometry "{}"
```

Resetting the odometry in CLI:
```
ros2 service call /ResetOdometry zuuu_interfaces/srv/ResetOdometry "{}"
```

### Follow me demo
Old code that requires a controller to be connected. Allows for controller control or automatic "follow me" behaviour.
This demo does not require the HAL to be started, just run: 
```
ros2 launch zuuu_follow_me follow_me_launch.py
```