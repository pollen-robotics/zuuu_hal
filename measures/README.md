The most basic linear model between PWM (between 0 and 1) and wheel rotation speed (rad/s) yields (based on measures, not on the model):
22.7 * PWM = wheel_rot_speed


![](2022-06-02-23-34-47.png)

![](2022-06-08-23-18-58.png)


Some quick measures :
10 full rotations measured in 31.3 sec, asking for a rotational speed of 2 rad/s PID mode. Good result (31.3 * 2 / (2*pi)) = 9.96.
10 full rotations measured in 31.41 sec, asking for a rotational speed of 2 rad/s OPEN_LOOP mode. Good result (31.3 * 2 / (2*pi)) = 10.00

using the set_speed service in OPEN_LOOP mode, asking for a rot speed of 2 rad/s for a duration of 31.4s gave an error of ~~30° (of excess after 10 full rotations). 

