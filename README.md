# Zuuu Hardware Abstraction Layer
## Installation
Clone this repository
```
git clone https://github.com/pollen-robotics/zuuu_follow_me.git
```

Clone and install **our custom version of PyVESC:**

https://github.com/pollen-robotics/PyVESC

```
pip3 install -e PyVESC/
```

Install:
```
sudo apt install ros-foxy-tf-transformations
pip3 install transforms3d
```

## Dependencies
* zuuu_interfaces: defines custom services for the mobile base
```
git clone https://github.com/pollen-robotics/zuuu_interfaces
```
* zuuu_description: if you want to use visualization tools (e.g. rviz, gazebo)

For the installation of this package, refer to the [dedicated README](https://github.com/pollen-robotics/zuuu_description).

## Usage
### Running the HAL
For all ROS based use cases, the zuuu_hal node must be running :
```
ros2 launch zuuu_hal hal_launch.py
```

### Sending speed commands
Once the zuuu_hal is started, one can take control of the mobile base with:
1) A controller
```
ros2 run zuuu_hal teleop_joy
```
2) A keyboard
```
ros2 run zuuu_hal teleop_keyboard
```
3) Sending speed commands on the 'cmd_vel' topic.
4) Using the [SetSpeed service](#setspeed-service)
5) Using the [GoToXYTheta service](#gotoxytheta-service)

### SetSpeed service
Sets a constant speed for a given duration. Zuuu should make a full rotation with this call:
```
ros2 service call /SetSpeed zuuu_interfaces/srv/SetSpeed "{x_vel: 0.0, y_vel: 0.0, rot_vel: 2.0, duration: 3.1415}"
```
This script makes Zuuu draw a 1x1 square (front, right, back, left):
```
ros2 run zuuu_hal set_speed_service_test
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
The goal is reached when the distance between the goal position and the position of the robot is less than the parameter xy_tol meters AND the angle error is less than theta_tol rads.
These parameters can be changed in the config file (recompiling might be needed to make any changes effective) or live through the ROS CLI interface:
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
:bulb: A good example where this setup is needed is if Zuuu needs to stay stationary on a slope.


### Odometry
Getting the odometry in CLI:
```
ros2 service call /GetOdometry zuuu_interfaces/srv/GetOdometry "{}"
```

Resetting the odometry in CLI:
```
ros2 service call /ResetOdometry zuuu_interfaces/srv/ResetOdometry "{}"
```
:warning: For safety reasons, if a call to ResetOdometry is made while the GoToXYTheta service is still trying to reach a destination, then the GoToXYTheta is stopped


:bulb: A visual test can be run using RViz. Setting a high value to the LIDAR decay time is a trick to visualize the errors of the odometry.

This launch file runs both the HAL, the lidar node and RViz:
```
ros2 launch zuuu_description zuuu_bringup.launch.py
```

Or, if the HAL is already running, then launch RViz only with:
```
ros2 launch zuuu_description rviz_bringup.launch.py
```


### Parameters
The parameter configuration file is in ```config/params.yaml```. 

Usage example to dynamically change the LIDAR angular limits:
```
ros2 param set /zuuu_hal laser_lower_angle -0.1
```

:warning: The node should always be run with its parameter file. The node will crash if the parameter file is not present at launch time (use the ```hal_launch.py``` launch file and it should work fine)

### Setting the drive mode
CMD_VEL is the default mode. Services will automatically change the drive mode as needed. 

:bulb: The most common use case where handling drive modes by hand is useful is when going back to CMD_VEL is needed after a call to SetSpeed or GoToXYTheta.

1. CMD_VEL = The commands read on the topic /cmd_vel are applied after smoothing

2. BRAKE =  Sets the PWMs to 0 effectively braking the base

3. FREE_WHEEL =  Sets the current control to 0, coast mode

4. SPEED =  Mode used by the set_speed service to do speed control over arbitrary duration

5. GOTO =  Mode used by the go_to_xytheta service to do position control in odom frame

6. EMERGENCY_STOP =  Calls the emergency_shutdown method

Can be tested with CLI:
```
ros2 service call /SetZuuuMode zuuu_interfaces/srv/SetZuuuMode "{mode: CMD_VEL}" 
ros2 service call /SetZuuuMode zuuu_interfaces/srv/SetZuuuMode "{mode: BRAKE}" 
ros2 service call /SetZuuuMode zuuu_interfaces/srv/SetZuuuMode "{mode: FREE_WHEEL}" 
ros2 service call /SetZuuuMode zuuu_interfaces/srv/SetZuuuMode "{mode: SPEED}" 
ros2 service call /SetZuuuMode zuuu_interfaces/srv/SetZuuuMode "{mode: GOTO}" 
ros2 service call /SetZuuuMode zuuu_interfaces/srv/SetZuuuMode "{mode: EMERGENCY_STOP}" 
```
:bulb: Having a terminal ready with the EMERGENCY_STOP service call is a fast way to have a software stop. After an emergency stop there is no need to restart the robot, just kill and relaunch the HAL.  



### Follow me demonstration
In this demo, Zuuu will follow anything that is in front of it using its LIDAR as the only sensor. This is only an usage example, as a robust follow me behaviour would require a more sophisticated approach.
By default you need to be between 30cm and 100cm to be detected. To change the default values, tweak range_min, range_max and detection_angle.

Start the HAL and the LIDAR:
```
ros2 launch zuuu_description zuuu_bringup_no_rviz.launch.py
```
And start the follow me node:
```
ros2 run zuuu_hal follow_me
```

### Misc
The Ziegler Nichols method was used to tune the PID values for the GoToXYTheta service. The result was way too dynamic so the parameters are tuned down.
If useful, here are the parameters found for the theta PID:

Ku = 27

Fu = 2.3 Hz => Tu = 0.4348 s

https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method


