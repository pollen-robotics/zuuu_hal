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
For all ROS based use cases, the zuuu_hal node must be running :
```
ros2 launch zuuu_follow_me hal_launch.py
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
You can run a visual test using RViz (setting a high value to the LIDAR decay time is a visual trick to see the integral of the errors of the odometry).
This launch file runs both the HAL, the lidar node and RViz:
```
ros2 launch zuuu_description zuuu_bringup.launch.py
```

Or, if the HAL is already running, then launch RViz only with:
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

### Parameters
The parameter configuration file is in ```config/params.yaml```. 
The node should always be run with its parameter file. 
To avoid dangerous situations with uninitialised parameters, the node will crash if the parameter file is not present at launch time (just use the launch file and it will link it)

Usage example to dynamically change the LIDAR angular limits:
```
ros2 param set /zuuu_hal laser_lower_angle -0.1
```

### Follow me demo
Old code that requires a controller to be connected. Allows for controller control or automatic "follow me" behaviour.
This demo does not require the HAL to be started, just run: 
```
ros2 launch zuuu_follow_me follow_me_launch.py
```