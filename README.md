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
Once the zuuu_hal is started, one can take control of the mobile base with:
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
ros2 service call /SetZuuuMode zuuu_interfaces/srv/SetZuuuMode "{mode: EMERGENCY_STOP}" 
```
-> Having a terminal ready with the EMERGENCY_STOP service call is a fast way to have a software stop. After an emergency stop, just kill and relaunch the HAL, no need to restart the robot. 

### Testing the odometry
A visual test can be run using RViz (setting a high value to the LIDAR decay time is a visual trick to see the integral of the errors of the odometry).
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

### SetSpeed service
Sets a constant speed for a given duration. Zuuu should make a full rotation with this call:
```
ros2 service call /SetSpeed zuuu_interfaces/srv/SetSpeed "{x_vel: 0.0, y_vel: 0.0, rot_vel: 2.0, duration: 3.1415}"
```
This script makes Zuuu draw a 1x1 square (front, right, back, left):
```
ros2 run zuuu_follow_me set_speed_service_test
```

### GoToXYTheta service
Sets a goal position in the odom frame. Zuuu will try to reach that position using 3 separate PIDs (one for x, one for y and one for theta).
Zuuu should go to position X=0.5, y=0.0, theta=0.0. If one resets the odom frame, this should be equivalent to moving forward 0.5m.
```
ros2 service call /ResetOdometry zuuu_interfaces/srv/ResetOdometry "{}"
ros2 service call /GoToXYTheta zuuu_interfaces/srv/GoToXYTheta "{x_goal: 0.5, y_goal: 0.0, theta_goal: 0.0}"
```

To check the current distance from the goal:
```
ros2 service call /DistanceToGoal zuuu_interfaces/srv/DistanceToGoal "{}"
```

Use the IsGoToFinished service to know if the goal position is reached:
```
ros2 service call /IsGoToFinished zuuu_interfaces/srv/IsGoToFinished "{}"
```
The goal is reached when the distance between the goal position and the position of the robot is less than the parameter xy_tol meters AND the angle error is less that theta_tol rads.
These parameters can be changed in the config file (recompiling might be needed to make any changes effective) or live through ROS CLI interface:
```
ros2 param set /zuuu_hal xy_tol 0.15
ros2 param set /zuuu_hal theta_tol 0.2
```
Once the goal is reached, the PWM values of the motors will be set to 0 until a new goal is received. This will brake Zuuu but the wheels can still be rotated if enough force is applied. 
To get a better disturbance rejection, the PIDs can always stay ON by applying unreachable tolerances:
```
ros2 param set /zuuu_hal xy_tol 0.0
ros2 param set /zuuu_hal theta_tol 0.0
```
-> A good example where this setup is needed is if Zuuu needs to stay stationary on a slope.



*Note:* The Ziegler Nichols method was tried to tune the PID values. The result was way too dynamic so the parameters have been tuned down.
If useful, here are the parameters found for the theta PID:
Ku = 27
Fu = 2.3 Hz => Tu = 0.4348 s

https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method


### Follow me demo
Old code that requires a controller to be connected. Allows for controller control or automatic "follow me" behaviour.
This demo does not require the HAL to be started, just run: 
```
ros2 launch zuuu_follow_me follow_me_launch.py
```


