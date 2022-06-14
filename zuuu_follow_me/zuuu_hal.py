import rclpy
from rclpy.node import Node
import time
import math
import numpy as np
import traceback
import sys
from enum import Enum
from pyvesc.VESC import MultiVESC
from example_interfaces.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.constants import S_TO_NS
from collections import deque
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from zuuu_interfaces.srv import SetZuuuMode, GetOdometry, ResetOdometry
from zuuu_interfaces.srv import GoToXYTheta, IsGoToFinished, DistanceToGoal
from zuuu_interfaces.srv import SetSpeed, GetBatteryVoltage
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from csv import writer
import tf_transformations
import copy


"""
This node has the responsability to interact with zuuu's hardware and 'ROSify' the inputs and outputs.
Zuuu's hardware here means 3 motor controllers and the LIDAR.
Specificaly, this node will periodically read the selected measurements from the controllers of the wheels (speed, temperature, IMUs, etc) and publish them into the adequate topics (odom, etc).
This node will also subscribe to /cmd_vel and write commands to the 3 motor controllers.
"""

"""
TODO :
(DONE) Modify zuuu_description to be able to launch the description without Gazebo
(DONE, PENDING TESTS) Connect the rest of the navigation stack
(DONE) Code functions for :
(DONE) brake()
(DONE) free_wheel()
(DONE) reset_odom()
(DONE) ik_vel is WRONG, give SI version and use the linear model to go from linear speed to PWM. Don't break old code. lin model is 22.7 * PWM = wheel_rot_speed
(DONE) do the usual fake moving average to smooth speed commands and limit the max accel with the parameter. Do this on X, Y and theta vel, not wheel speed.
(DONE)service for set_speed(x_speed, y_speed, rot_speed, mode=open_loop, max_accel_xy=, max_accel_theta=, duration=)
      -> will do a service with x, y, theta and duration all mandatory. The rest become ROS parameters.
(DONE) go_to(x, y, theta)
(DONE) PID as parameters

- Read IMU (not supported by PyVesc out of the box). Hints: create a message (examples in getters.py) with "VedderCmd.COMM_GET_IMU_DATA".
The unknowns are the field names and their types. Maybe the "IMU_VALUES" in the file "datatypes.h" in vesc_project.
"""

# Interesting stuff can be found in Modern Robotics' chapters:
# 13.2 Omnidirectional Wheeled Mobile Robots
# 13.4 Odometry
# /!\ Our robot frame is different. Matching between their names (left) and ours (right): xb=y, yb=-x, theta=-theta, u1=uB, u2=uL, u3=uR

SAVE_CSV = False


# utility functions

def angle_diff(a, b):
    """Returns the smallest distance between 2 angles
    """
    d = a - b
    d = ((d + math.pi) % (2 * math.pi)) - math.pi
    return d


def sign(x):
    """Returns 1 if x >= 0, -1 otherwise
    """
    if x >= 0:
        return 1
    else:
        return -1


class ZuuuModes(Enum):
    """
    Zuuu drive modes
    CMD_VEL = The commands read on the topic /cmd_vel are applied after smoothing
    BRAKE =  Sets the PWMs to 0 effectively braking the base
    FREE_WHEEL =  Sets the current control to 0, coast mode
    SPEED =  Mode used by the set_speed service to do speed control over arbitrary duration
    GOTO =  Mode used by the go_to_xytheta service to do position control in odom frame
    EMERGENCY_STOP =  Calls the emergency_shutdown method
    """
    CMD_VEL = 1
    BRAKE = 2
    FREE_WHEEL = 3
    SPEED = 4
    GOTO = 5
    EMERGENCY_STOP = 6


class ZuuuControlModes(Enum):
    """Zuuu control modes"""
    OPEN_LOOP = 1
    PID = 2


class PID:
    def __init__(self, P=1.0, I=0.0, D=0.0, max_command=10, max_i_contribution=5):
        """PID implementation with anti windup.
        Keyword Arguments:
            P {float} -- Proportional gain (default: {1.0})
            I {float} -- Integral gain (default: {0.0})
            D {float} -- Differential gain (default: {0.0})
            max_command {float} -- The output command will be trimmed to +- max_command (default: {10})
            max_i_contribution {float} -- The integral contribution will be trimmed to +- max_i_contribution (default: {5})
        """
        self.p = P
        self.i = I
        self.d = D
        self.max_command = max_command
        self.max_i_contribution = max_i_contribution
        self.goal_value = 0
        self.current_value = 0

        self.prev_error = 0
        self.i_contribution = 0
        self.prev_t = time.time()

    def set_goal(self, goal_value):
        """Sets the goal state
        """
        self.goal_value = goal_value
        # Reseting the persistent data because the goal state changed
        self.reset()

    def reset(self):
        """Resets the integral portion, dt and the differential contribution
        """
        self.i_contribution = 0
        self.prev_t = time.time()
        self.prev_error = self.goal_value - self.current_value

    def limit(self, x, limit):
        if x > abs(limit):
            return abs(limit)
        elif x < -abs(limit):
            return -abs(limit)
        else:
            return x

    def tick(self, current_value, is_angle=False):
        """PID calculations. If is_angle is True, then the error will be calculated as the smallest angle between the goal and the current_value
        Arguments:
            current_value {float} -- Current state, usually the feedback value

        Returns:
            float -- The output command
        """
        self.current_value = current_value
        if is_angle:
            error = angle_diff(self.goal_value, self.current_value)
        else:
            error = self.goal_value - self.current_value
        t = time.time()
        dt = t - self.prev_t
        self.prev_t = t
        delta_error = error - self.prev_error
        self.prev_error = error

        p_contribution = self.p * error
        if dt != 0:
            d_contribution = self.d * delta_error / dt
        else:
            d_contribution = 0.0

        self.i_contribution = self.i_contribution + self.i * error
        self.i_contribution = self.limit(
            self.i_contribution, self.max_i_contribution)

        self.command = p_contribution + self.i_contribution + d_contribution
        self.command = self.limit(self.command, self.max_command)

        return self.command


class MobileBase:
    def __init__(
        self,
        serial_port='/dev/vesc_wheels',
        left_wheel_id=24,
        right_wheel_id=72,
        back_wheel_id=None,
    ) -> None:

        params = [
            {'can_id': left_wheel_id, 'has_sensor': True, 'start_heartbeat': True},
            {'can_id': right_wheel_id, 'has_sensor': True, 'start_heartbeat': True},
            {'can_id': back_wheel_id, 'has_sensor': True, 'start_heartbeat': True},
        ]
        self._multi_vesc = MultiVESC(
            serial_port=serial_port, vescs_params=params)

        self.left_wheel, self.right_wheel, self.back_wheel = self._multi_vesc.controllers
        self.left_wheel_measurements, self.right_wheel_measurements, self.back_wheel_measurements = None, None, None
        self.left_wheel_nones, self.right_wheel_nones, self.back_wheel_nones = 0, 0, 0
        self.wheel_radius = 0.21/2.0
        self.wheel_to_center = 0.19588  # 0.188
        self.half_poles = 15.0
        self.left_wheel_rpm, self.right_wheel_rpm, self.back_wheel_rpm = 0, 0, 0
        self.left_wheel_avg_rpm, self.right_wheel_avg_rpm, self.back_wheel_avg_rpm = 0, 0, 0
        self.left_wheel_rpm_deque, self.right_wheel_rpm_deque, self.back_wheel_rpm_deque = deque(
            [], 10), deque([], 10), deque([], 10)
        # These values might be a tad too safe, however the battery should be almost empty when the cells are on average at 3.3V so there is little
        # to win to go below this. Still tunable if needed.
        # The current battery has a BMS that shuts down the battery at 20V +-1V. So that would be 2.86V +-0.14V.
        self.battery_cell_warn_voltage = 3.5
        self.battery_cell_min_voltage = 3.3
        self.battery_nb_cells = 7
        self.battery_check_period = 2

    def read_all_measurements(self):
        self.left_wheel_measurements = self.left_wheel.get_measurements()
        self.right_wheel_measurements = self.right_wheel.get_measurements()
        self.back_wheel_measurements = self.back_wheel.get_measurements()

    def deque_to_avg(self, deque):
        sum = 0
        len = deque.maxlen
        for i in deque:
            sum += i
        return sum/float(len)


class ZuuuHAL(Node):
    def __init__(self):
        super().__init__('zuuu_hal')
        if SAVE_CSV:
            self.csv_file = open('zuuu_data.csv', 'a+')
            self.csv_writer = writer(self.csv_file)
            self.csv_writer.writerow(
                ["Time (s)", "PWM (%)", "Wheel speed (rad/s)", "zuuu rot speed (rad/s)"])
        self.get_logger().info("Starting zuuu_hal!")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('laser_upper_angle', None),
                ('laser_lower_angle', None),
                ('max_duty_cyle', None),
                ('cmd_vel_timeout', None),
                ('max_full_com_fails', None),
                ('main_tick_period', None),
                ('control_mode', None),
                ('max_accel_xy', None),
                ('max_accel_theta', None),
                ('xy_tol', None),
                ('theta_tol', None),
                ('smoothing_factor', None),
            ])
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Maing sure we don't run the node if the config file is not shared
        if self.get_parameter('max_duty_cyle').type_ is Parameter.Type.NOT_SET:
            self.get_logger().error(
                f"Parameter 'max_duty_cyle' was not initialized. Check that the param file is given to the node (using the launch file is the way to go). Shutting down")
            self.destroy_node()
        if self.get_parameter('smoothing_factor').type_ is Parameter.Type.NOT_SET:
            self.get_logger().error(
                f"Parameter 'smoothing_factor' was not initialized. Check that the param file is given to the node (using the launch file is the way to go). Shutting down")
            self.destroy_node()

        # Parameters initialisation
        self.laser_upper_angle = self.get_parameter(
            'laser_upper_angle').get_parameter_value().double_value  # math.pi
        self.laser_lower_angle = self.get_parameter(
            'laser_lower_angle').get_parameter_value().double_value  # -math.pi
        self.max_duty_cyle = self.get_parameter(
            'max_duty_cyle').get_parameter_value().double_value  # 0.3  # max is 1
        self.cmd_vel_timeout = self.get_parameter(
            'cmd_vel_timeout').get_parameter_value().double_value  # 0.2
        self.max_full_com_fails = self.get_parameter(
            'max_full_com_fails').get_parameter_value().integer_value  # 100
        self.main_tick_period = self.get_parameter(
            'main_tick_period').get_parameter_value().double_value  # 0.012

        control_mode_param = self.get_parameter('control_mode')
        if control_mode_param.value in [m.name for m in ZuuuControlModes]:
            # "OPEN_LOOP"
            self.control_mode = ZuuuControlModes[control_mode_param.value]
        else:
            self.get_logger().error(
                f"Parameter 'control_mode' has an unknown value: '{control_mode_param.value}'. Shutting down")
            self.destroy_node()

        self.max_accel_xy = self.get_parameter(
            'max_accel_xy').get_parameter_value().double_value  # 1.0
        self.max_accel_theta = self.get_parameter(
            'max_accel_theta').get_parameter_value().double_value  # 1.0
        self.xy_tol = self.get_parameter(
            'xy_tol').get_parameter_value().double_value  # 0.2
        self.theta_tol = self.get_parameter(
            'theta_tol').get_parameter_value().double_value  # 0.17
        self.smoothing_factor = self.get_parameter(
            'smoothing_factor').get_parameter_value().double_value  # 100.0

        self.cmd_vel = None
        self.x_odom = 0.0
        self.y_odom = 0.0
        self.theta_odom = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        self.x_vel = 0.0
        self.y_vel = 0.0
        self.theta_vel = 0.0
        self.x_vel_goal = 0.0
        self.y_vel_goal = 0.0
        self.theta_vel_goal = 0.0
        self.x_vel_goal_filtered = 0.0
        self.y_vel_goal_filtered = 0.0
        self.theta_vel_goal_filtered = 0.0
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.theta_goal = 0.0
        self.reset_odom = False
        self.battery_voltage = 25.0
        self.mode = ZuuuModes.CMD_VEL
        self.speed_service_deadline = 0
        self.speed_service_on = False
        self.goto_service_on = False
        self.x_pid = PID(P=3.0, I=0.00, D=0.0, max_command=0.5,
                         max_i_contribution=0.0)
        self.y_pid = PID(P=3.0, I=0.00, D=0.0, max_command=0.5,
                         max_i_contribution=0.0)
        self.theta_pid = PID(P=3.0, I=0.0, D=0.0,
                             max_command=1.6, max_i_contribution=0.0)

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.cmd_vel_sub  # prevent unused variable warning... JESUS WHAT HAVE WE BECOME

        # TODO Temporary fix since https://github.com/ros-perception/laser_filters doesn't work on Foxy aparently
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_filter_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.scan_sub  # prevent unused variable warning... JESUS WHAT HAVE WE BECOME
        self.scan_pub = self.create_publisher(
            LaserScan, 'scan_filterd', 10)

        self.pub_back_wheel_rpm = self.create_publisher(
            Float32, 'back_wheel_rpm', 2)
        self.pub_left_wheel_rpm = self.create_publisher(
            Float32, 'left_wheel_rpm', 2)
        self.pub_right_wheel_rpm = self.create_publisher(
            Float32, 'right_wheel_rpm', 2)

        self.pub_odom = self.create_publisher(
            Odometry, 'odom', 2)

        self.mode_service = self.create_service(
            SetZuuuMode, 'SetZuuuMode', self.handle_zuuu_mode)

        self.reset_odometry_service = self.create_service(
            ResetOdometry, 'ResetOdometry', self.handle_reset_odometry)

        self.get_odometry_service = self.create_service(
            GetOdometry, 'GetOdometry', self.handle_get_odometry)

        self.set_speed_service = self.create_service(
            SetSpeed, 'SetSpeed', self.handle_set_speed)

        # I chose not to make an action client. Could be changed if needed.
        self.go_to_service = self.create_service(
            GoToXYTheta, 'GoToXYTheta', self.handle_go_to)

        self.is_go_to_finished = self.create_service(
            IsGoToFinished, 'IsGoToFinished', self.handle_is_go_to_finished)

        self.distance_to_goal = self.create_service(
            DistanceToGoal, 'DistanceToGoal', self.handle_distance_to_goal)

        self.get_battery_voltage_service = self.create_service(
            GetBatteryVoltage, 'GetBatteryVoltage', self.handle_get_battery_voltage)

        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)

        self.omnibase = MobileBase()

        self.old_measure_timestamp = self.get_clock().now()
        self.measure_timestamp = self.get_clock().now()
        # *sigh* if needed use: https://github.com/ros2/rclpy/blob/master/rclpy/test/test_time.py
        self.cmd_vel_t0 = time.time()
        self.t0 = time.time()
        self.get_logger().info(
            "Reading Zuuu's sensors once...")
        self.read_measurements()
        self.get_logger().info(
            "zuuu_hal started, you can write to cmd_vel to move the robot")
        self.get_logger().info("List of published topics: TODO")

        self.create_timer(self.main_tick_period, self.main_tick)
        # self.create_timer(0.1, self.main_tick)
        self.measurements_t = time.time()
        self.create_timer(self.omnibase.battery_check_period,
                          self.check_battery)

    def parameters_callback(self, params):
        success = False
        for param in params:
            if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                if param.name == "laser_upper_angle":
                    self.laser_upper_angle = param.value
                    success = True
                elif param.name == "laser_lower_angle":
                    self.laser_lower_angle = param.value
                    success = True
                elif param.name == "max_duty_cyle":
                    if param.value >= 0.0 and param.value <= 1.0:
                        self.max_duty_cyle = param.value
                        success = True
                elif param.name == "cmd_vel_timeout":
                    if param.value >= 0.0:
                        self.cmd_vel_timeout = param.value
                        success = True
                elif param.name == "max_full_com_fails":
                    if param.value >= 0.0:
                        self.max_full_com_fails = param.value
                        success = True
                elif param.name == "main_tick_period":
                    if param.value >= 0.0:
                        self.main_tick_period = param.value
                        success = True
                elif param.name == "max_accel_xy":
                    if param.value >= 0.0:
                        self.max_accel_xy = param.value
                        success = True
                elif param.name == "max_accel_theta":
                    if param.value >= 0.0:
                        self.max_accel_theta = param.value
                        success = True
                elif param.name == "xy_tol":
                    if param.value >= 0.0:
                        self.xy_tol = param.value
                        success = True
                elif param.name == "theta_tol":
                    if param.value >= 0.0:
                        self.theta_tol = param.value
                        success = True
                elif param.name == "smoothing_factor":
                    if param.value >= 0.0:
                        self.smoothing_factor = param.value
                        success = True
            elif param.type_ is Parameter.Type.STRING:
                if param.name == "control_mode":
                    if param.value in [m.name for m in ZuuuControlModes]:
                        self.control_mode = ZuuuControlModes[param.value]
                        success = True

        return SetParametersResult(successful=success)

    def handle_zuuu_mode(self, request, response):
        mode = request.mode
        self.get_logger().info("Requested mode change to :'{}'".format(mode))

        if mode in [m.name for m in ZuuuModes]:
            self.mode = ZuuuModes[mode]
            response.success = True
            if self.mode is not ZuuuModes.SPEED and self.mode is not ZuuuModes.GOTO:
                # Changing the mode is a way to prematurely end an on going task requested through a service
                self.stop_ongoing_services()
        else:
            response.success = False
        return response

    def handle_reset_odometry(self, request, response):
        # Resetting asynchronously to prevent race conditions.
        self.reset_odom = True
        self.get_logger().info("Requested to reset the odometry frame")
        response.success = True
        return response

    def handle_get_odometry(self, request, response):
        response.x = self.x_odom
        response.y = self.y_odom
        response.theta = self.theta_odom
        return response

    def handle_set_speed(self, request, response):
        # This service automatically changes the zuuu mode
        self.mode = ZuuuModes.SPEED
        self.get_logger().info(
            f"Requested set_speed: duration={request.duration} x_vel='{request.x_vel}'m/s, y_vel='{request.y_vel}'m/s, rot_vel='{request.rot_vel}'rad/s")
        self.x_vel_goal = request.x_vel
        self.y_vel_goal = request.y_vel
        self.theta_vel_goal = request.rot_vel
        self.speed_service_deadline = time.time() + request.duration
        self.speed_service_on = True
        response.success = True
        return response

    def handle_go_to(self, request, response):
        # This service automatically changes the zuuu mode
        self.mode = ZuuuModes.GOTO
        self.get_logger().info(
            f"Requested go_to: x={request.x_goal}m, y={request.y_goal}m, theta={request.theta_goal}rad")
        self.x_goal = request.x_goal
        self.y_goal = request.y_goal
        self.theta_goal = request.theta_goal
        self.goto_service_on = True
        self.x_pid.set_goal(self.x_goal)
        self.y_pid.set_goal(self.y_goal)
        self.theta_pid.set_goal(self.theta_goal)
        response.success = True
        return response

    def handle_is_go_to_finished(self, request, response):
        # Returns True if the goal position is reached
        response.success = self.goto_service_on
        return response

    def handle_distance_to_goal(self, request, response):
        response.delta_x = self.x_goal - self.x_odom
        response.delta_y = self.y_goal - self.y_odom
        response.delta_theta = angle_diff(self.theta_goal, self.theta_odom)
        response.distance = math.sqrt(
            (self.x_goal - self.x_odom)**2 +
            (self.y_goal - self.y_odom)**2)
        return response

    def handle_get_battery_voltage(self, request, response):
        response.voltage = self.battery_voltage
        return response

    def check_battery(self, verbose=False):
        t = time.time()
        if verbose:
            self.print_all_measurements()
        if (t - self.measurements_t) > (self.omnibase.battery_check_period+1):
            self.get_logger().warning("Zuuu's measurements are not made often enough. Reading now.")
            self.read_measurements()
        warn_voltage = self.omnibase.battery_nb_cells * \
            self.omnibase.battery_cell_warn_voltage
        min_voltage = self.omnibase.battery_nb_cells * \
            self.omnibase.battery_cell_min_voltage
        voltage = self.battery_voltage

        if (min_voltage < voltage < warn_voltage):
            self.get_logger().warning("Battery voltage LOW ({}V). Consider recharging. Warning threshold: {}V, stop threshold: {}V".format(
                voltage, warn_voltage, min_voltage))
        elif (voltage < min_voltage):
            self.get_logger().error("Battery voltage critically LOW ({}V). Emergency shutdown! Warning threshold: {}V, stop threshold: {}V".format(
                voltage, warn_voltage, min_voltage))
            self.emergency_shutdown()
        else:
            pass
            self.get_logger().warning("Battery voltage OK ({}V). Warning threshold: {}V, stop threshold: {}V".format(
                voltage, warn_voltage, min_voltage))

    def emergency_shutdown(self):
        self.omnibase.back_wheel.set_duty_cycle(0)
        self.omnibase.left_wheel.set_duty_cycle(0)
        self.omnibase.right_wheel.set_duty_cycle(0)
        if SAVE_CSV:
            self.csv_file.close()
        self.get_logger().warn("Emergency shutdown!")
        sys.exit(1)

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg
        self.cmd_vel_t0 = time.time()

    def scan_filter_callback(self, msg):
        filtered_scan = LaserScan()
        filtered_scan.header = copy.deepcopy(msg.header)
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        ranges = []
        intensities = []
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i*msg.angle_increment
            if angle > self.laser_upper_angle or angle < self.laser_lower_angle:
                ranges.append(0.0)
                intensities.append(0.0)
            else:
                ranges.append(r)
                intensities.append(msg.intensities[i])
        filtered_scan.ranges = ranges
        filtered_scan.intensities = intensities
        self.scan_pub.publish(filtered_scan)

    def wheel_rot_speed_to_pwm_no_friction(self, rot):
        """Uses a simple linear model to map the expected rotational speed of the wheel to a constant PWM (based on measures made on a full Reachy Mobile)
        """
        return rot/22.7

    def wheel_rot_speed_to_pwm(self, rot):
        """Uses a simple affine model to map the expected rotational speed of the wheel to a constant PWM (based on measures made on a full Reachy Mobile)
        """
        # Creating an arteficial null zone to avoid undesired behaviours for very small rot speeds
        epsilon = 0.02
        if rot > epsilon:
            pwm = 0.0418 * rot + 0.0126
        elif rot < -epsilon:
            pwm = 0.0418 * rot - 0.0126
        else:
            pwm = 0.0

        return pwm

    def ik_vel_to_pwm(self, x_vel, y_vel, rot_vel):
        rot_vels = self.ik_vel(x_vel, y_vel, rot_vel)
        return [self.wheel_rot_speed_to_pwm(rot_vel) for rot_vel in rot_vels]

    def ik_vel_old(self, x, y, rot):
        """Takes 2 linear speeds and 1 rot speed (robot's egocentric frame) and outputs the PWM to apply to each of the 3 motors in an omni setup

        Args:
            x (float): x speed (between -1 and 1). Positive "in front" of the robot.
            y (float): y speed (between -1 and 1). Positive "to the left" of the robot.
            rot (float): rotational speed (between -1 and 1). Positive counter-clock wise.
        """

        cycle_back = -y + rot
        cycle_right = (-y*np.cos(120*np.pi/180)) + \
            (x*np.sin(120*np.pi/180)) + rot
        cycle_left = (-y*np.cos(240*np.pi/180)) + \
            (x*np.sin(240*np.pi/180)) + rot

        return [cycle_back, cycle_right, cycle_left]

    def ik_vel(self, x_vel, y_vel, rot_vel):
        """Takes 2 linear speeds and 1 rot speed (robot's egocentric frame) and outputs the rotational speed (rad/s) of each of the 3 motors in an omni setup

        Args:
            x (float): x speed (m/s). Positive "in front" of the robot.
            y (float): y speed (m/s). Positive "to the left" of the robot.
            rot (float): rotational speed (rad/s). Positive counter-clock wise.
        """

        wheel_rot_speed_back = (1/self.omnibase.wheel_radius) * \
            (self.omnibase.wheel_to_center*rot_vel - y_vel)
        wheel_rot_speed_right = (1/self.omnibase.wheel_radius) * (
            self.omnibase.wheel_to_center*rot_vel + y_vel/2.0 + math.sin(math.pi/3)*x_vel)
        wheel_rot_speed_left = (1/self.omnibase.wheel_radius) * (self.omnibase.wheel_to_center *
                                                                 rot_vel + math.sin(math.pi/3)*y_vel/2 - math.sin(math.pi/3)*x_vel)

        return [wheel_rot_speed_back, wheel_rot_speed_right, wheel_rot_speed_left]

    def dk_vel(self, rot_l, rot_r, rot_b):
        """Takes the 3 rotational speeds (in rpm) of the 3 wheels and outputs the x linear speed (m/s), y linear speed (m/s) and rotational speed (rad/s)
        in the robot egocentric frame

        Args:
            rot_l (float): rpm speed of the left wheel
            rot_r (float): rpm speed of the right wheel
            rot_b (float): rpm speed of the back wheel
        """
        # rpm to rad/s then m/s
        speed_l = (2*math.pi*rot_l/60)*self.omnibase.wheel_radius
        speed_r = (2*math.pi*rot_r/60)*self.omnibase.wheel_radius
        speed_b = (2*math.pi*rot_b/60)*self.omnibase.wheel_radius

        x_vel = -speed_l*(1/(2*math.sin(math.pi/3))) + \
            speed_r*(1/(2*math.sin(math.pi/3)))
        y_vel = -speed_b*2/3.0 + speed_l*1/3.0 + speed_r*1/3.0
        theta_vel = (speed_l + speed_r + speed_b) / \
            (3*self.omnibase.wheel_to_center)

        return [x_vel, y_vel, theta_vel]

    def filter_speed_goals(self):
        self.x_vel_goal_filtered = (self.x_vel_goal +
                                    self.smoothing_factor*self.x_vel_goal_filtered)/(1+self.smoothing_factor)
        self.y_vel_goal_filtered = (self.y_vel_goal +
                                    self.smoothing_factor*self.y_vel_goal_filtered)/(1+self.smoothing_factor)
        self.theta_vel_goal_filtered = (self.theta_vel_goal +
                                        self.smoothing_factor*self.theta_vel_goal_filtered)/(1+self.smoothing_factor)

    def format_measurements(self, measurements):
        if measurements is None:
            return "None"
        to_print = ""
        to_print += "temp_fet:{}\n".format(measurements.temp_fet)
        to_print += "temp_motor:{}\n".format(measurements.temp_motor)
        to_print += "avg_motor_current:{}\n".format(
            measurements.avg_motor_current)
        to_print += "avg_input_current:{}\n".format(
            measurements.avg_input_current)
        to_print += "avg_id:{}\n".format(measurements.avg_id)
        to_print += "avg_iq:{}\n".format(measurements.avg_iq)
        to_print += "duty_cycle_now:{}\n".format(measurements.duty_cycle_now)
        to_print += "rpm:{}\n".format(measurements.rpm)
        to_print += "v_in:{}\n".format(measurements.v_in)
        to_print += "amp_hours:{}\n".format(measurements.amp_hours)
        to_print += "amp_hours_charged:{}\n".format(
            measurements.amp_hours_charged)
        to_print += "watt_hours:{}\n".format(measurements.watt_hours)
        to_print += "watt_hours_charged:{}\n".format(
            measurements.watt_hours_charged)
        to_print += "tachometer:{}\n".format(measurements.tachometer)
        to_print += "tachometer_abs:{}\n".format(measurements.tachometer_abs)
        to_print += "mc_fault_code:{}\n".format(measurements.mc_fault_code)
        to_print += "pid_pos_now:{}\n".format(measurements.pid_pos_now)
        to_print += "app_controller_id:{}\n".format(
            measurements.app_controller_id)
        to_print += "time_ms:{}\n".format(measurements.time_ms)
        return to_print

    def print_all_measurements(self):
        to_print = "\n*** back_wheel measurements:\n"
        to_print += self.format_measurements(
            self.omnibase.back_wheel_measurements)
        to_print += "\n\n*** left_wheel:\n"
        to_print += self.format_measurements(
            self.omnibase.left_wheel_measurements)
        to_print += "\n\n*** right_wheel:\n"
        to_print += self.format_measurements(
            self.omnibase.right_wheel_measurements)
        to_print += "\n\n Fails ('Nones') left:{}, right:{}, back:{}".format(
            self.omnibase.left_wheel_nones, self.omnibase.right_wheel_nones, self.omnibase.back_wheel_nones)
        to_print += "\n\n AVG RPM left:{:.2f}, right:{:.2f}, back:{:.2f}".format(
            self.omnibase.left_wheel_avg_rpm/self.omnibase.half_poles, self.omnibase.right_wheel_avg_rpm/self.omnibase.half_poles, self.omnibase.back_wheel_avg_rpm/self.omnibase.half_poles)

        self.get_logger().info("{}".format(to_print))
        # 20 tours en 35s, avg_rpm ~=34

    def publish_wheel_speeds(self):
        # If the measurements are None, not publishing
        if self.omnibase.back_wheel_measurements is not None:
            rpm_back = Float32()
            rpm_back.data = float(self.omnibase.back_wheel_measurements.rpm)
            self.pub_back_wheel_rpm.publish(rpm_back)

        if self.omnibase.left_wheel_measurements is not None:
            rpm_left = Float32()
            rpm_left.data = float(self.omnibase.left_wheel_measurements.rpm)
            self.pub_left_wheel_rpm.publish(rpm_left)

        if self.omnibase.right_wheel_measurements is not None:
            rpm_right = Float32()
            rpm_right.data = float(self.omnibase.right_wheel_measurements.rpm)
            self.pub_right_wheel_rpm.publish(rpm_right)

    def update_wheel_speeds(self):
        # Keeping a local value of the wheel speeds to handle None measurements (we'll use the last valid measure)
        if self.omnibase.back_wheel_measurements is not None:
            value = float(
                self.omnibase.back_wheel_measurements.rpm)
            self.omnibase.back_wheel_rpm = value
            self.omnibase.back_wheel_rpm_deque.appendleft(value)
            self.omnibase.back_wheel_avg_rpm = self.omnibase.deque_to_avg(
                self.omnibase.back_wheel_rpm_deque)
        else:
            self.omnibase.back_wheel_nones += 1

        if self.omnibase.left_wheel_measurements is not None:
            value = float(
                self.omnibase.left_wheel_measurements.rpm)
            self.omnibase.left_wheel_rpm = value
            self.omnibase.left_wheel_rpm_deque.appendleft(value)
            self.omnibase.left_wheel_avg_rpm = self.omnibase.deque_to_avg(
                self.omnibase.left_wheel_rpm_deque)
        else:
            self.omnibase.left_wheel_nones += 1

        if self.omnibase.right_wheel_measurements is not None:
            value = float(
                self.omnibase.right_wheel_measurements.rpm)
            self.omnibase.right_wheel_rpm = value
            self.omnibase.right_wheel_rpm_deque.appendleft(value)
            self.omnibase.right_wheel_avg_rpm = self.omnibase.deque_to_avg(
                self.omnibase.right_wheel_rpm_deque)
        else:
            self.omnibase.right_wheel_nones += 1

    def publish_odometry_and_tf(self):
        # Odom
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.header.stamp = self.measure_timestamp.to_msg()
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x_odom
        odom.pose.pose.position.y = self.y_odom
        odom.pose.pose.position.z = 0.0
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vtheta

        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.theta_odom)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # TODO tune these numbers if needed
        odom.pose.covariance = np.diag(
            [1e-2, 1e-2, 1e-2, 1e3, 1e3, 1e-1]).ravel()
        odom.twist.twist.linear.x = self.x_vel
        odom.twist.twist.linear.y = self.y_vel
        odom.twist.twist.angular.z = self.theta_vel

        odom.twist.covariance = np.diag(
            [1e-2, 1e3, 1e3, 1e3, 1e3, 1e-2]).ravel()
        self.pub_odom.publish(odom)

        # TF
        t = TransformStamped()
        t.header.stamp = self.measure_timestamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.x_odom
        t.transform.translation.y = self.y_odom
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)

    def tick_odom(self):
        # TODO VESC speed values are, as is normal, very noisy at low speeds. We have no control on how the speeds are calculated as is.
        # -> By reading the encoder ticks directly and making the calculations here we could make this a tad better.

        # Local speeds in egocentric frame. Care, "rpm" are actually erpm and need to be divided by half the amount of magnetic poles to get the actual rpm.
        self.x_vel, self.y_vel, self.theta_vel = self.dk_vel(self.omnibase.left_wheel_rpm/self.omnibase.half_poles,
                                                             self.omnibase.right_wheel_rpm/self.omnibase.half_poles, self.omnibase.back_wheel_rpm/self.omnibase.half_poles)
        # self.get_logger().info(
        #     "IK vel : {:.2f}, {:.2f}, {:.2f}".format(self.x_vel, self.y_vel, self.theta_vel))
        # Applying the small displacement in the world-fixed odom frame (simple 2D rotation)
        dt_duration = (self.measure_timestamp - self.old_measure_timestamp)
        dt_seconds = dt_duration.nanoseconds/S_TO_NS
        dx = (self.x_vel * math.cos(self.theta_odom) - self.y_vel *
              math.sin(self.theta_odom)) * dt_seconds
        dy = (self.x_vel * math.sin(self.theta_odom) + self.y_vel *
              math.cos(self.theta_odom)) * dt_seconds
        dtheta = self.theta_vel*dt_seconds
        self.x_odom += dx
        self.y_odom += dy
        self.theta_odom += dtheta

        self.vx = dx / dt_seconds
        self.vy = dy / dt_seconds
        self.vtheta = dtheta / dt_seconds
        if self.reset_odom:
            # Resetting asynchronously to prevent race conditions.
            # dx, dy and dteta remain correct even on the reset tick
            self.reset_odom = False
            self.x_odom = 0.0
            self.y_odom = 0.0
            self.theta_odom = 0.0

        self.publish_odometry_and_tf()

    def limit_duty_cycles(self, duty_cycles):
        # Between +- max_duty_cyle
        for i in range(len(duty_cycles)):
            if duty_cycles[i] < 0:
                duty_cycles[i] = max(-self.max_duty_cyle, duty_cycles[i])
            else:
                duty_cycles[i] = min(self.max_duty_cyle, duty_cycles[i])
        return duty_cycles

    def read_measurements(self):
        self.omnibase.read_all_measurements()
        if self.omnibase.back_wheel_measurements is not None:
            self.battery_voltage = self.omnibase.back_wheel_measurements.v_in
        elif self.omnibase.left_wheel_measurements is not None:
            self.battery_voltage = self.omnibase.left_wheel_measurements.v_in
        elif self.omnibase.right_wheel_measurements is not None:
            self.battery_voltage = self.omnibase.right_wheel_measurements.v_in
        else:
            # Decidemment ! Keeping last valid measure...
            self.nb_full_com_fails += 1
            self.get_logger().warning(
                "Could not read any of the motor drivers. This should not happen often.")
            if (self.nb_full_com_fails > self.max_full_com_fails):
                self.get_logger().error("Too many communication errors, emergency shutdown")
                self.emergency_shutdown()
            return
        # Read success
        self.nb_full_com_fails = 0
        self.measurements_t = time.time()

    def send_wheel_commands(self, wheel_speeds):
        if self.control_mode is ZuuuControlModes.OPEN_LOOP:
            duty_cycles = [self.wheel_rot_speed_to_pwm(
                wheel_speed) for wheel_speed in wheel_speeds]
            duty_cycles = self.limit_duty_cycles(duty_cycles)
            self.omnibase.back_wheel.set_duty_cycle(
                duty_cycles[0])
            self.omnibase.left_wheel.set_duty_cycle(
                duty_cycles[2])
            self.omnibase.right_wheel.set_duty_cycle(
                duty_cycles[1])
        elif self.control_mode is ZuuuControlModes.PID:
            # rad/s to rpm to erpm
            self.omnibase.back_wheel.set_rpm(
                int(self.omnibase.half_poles*wheel_speeds[0]*30/math.pi))
            self.omnibase.left_wheel.set_rpm(
                int(self.omnibase.half_poles*wheel_speeds[2]*30/math.pi))
            self.omnibase.right_wheel.set_rpm(
                int(self.omnibase.half_poles*wheel_speeds[1]*30/math.pi))
        else:
            self.get_logger().warning("unknown control mode '{}'".format(self.control_mode))

    def position_control(self):
        x_command = self.x_pid.tick(self.x_odom)
        y_command = self.y_pid.tick(self.y_odom)
        theta_command = self.theta_pid.tick(self.theta_odom, is_angle=True)

        return x_command, y_command, theta_command

    def stop_ongoing_services(self):
        self.goto_service_on = False
        self.speed_service_on = False

    def main_tick(self, verbose=False):
        duty_cycles = [0, 0, 0]
        t = time.time()
        # Actually sending the commands
        if verbose:
            self.get_logger().info("cycles : {}".format(duty_cycles))

        if self.mode is ZuuuModes.CMD_VEL:
            # If too much time without an order, the speeds are smoothed back to 0 for safety.
            if (self.cmd_vel is not None) and ((t - self.cmd_vel_t0) < self.cmd_vel_timeout):
                self.x_vel_goal = self.cmd_vel.linear.x
                self.y_vel_goal = self.cmd_vel.linear.y
                self.theta_vel_goal = self.cmd_vel.angular.z
            else:
                self.x_vel_goal = 0.0
                self.y_vel_goal = 0.0
                self.theta_vel_goal = 0.0
            self.filter_speed_goals()
            wheel_speeds = self.ik_vel(
                self.x_vel_goal_filtered, self.y_vel_goal_filtered, self.theta_vel_goal_filtered)
            self.send_wheel_commands(wheel_speeds)
        elif self.mode is ZuuuModes.BRAKE:
            self.omnibase.back_wheel.set_duty_cycle(0)
            self.omnibase.left_wheel.set_duty_cycle(0)
            self.omnibase.right_wheel.set_duty_cycle(0)
        elif self.mode is ZuuuModes.FREE_WHEEL:
            self.omnibase.back_wheel.set_current(0)
            self.omnibase.left_wheel.set_current(0)
            self.omnibase.right_wheel.set_current(0)
        elif self.mode is ZuuuModes.SPEED:
            if self.speed_service_deadline < time.time():
                if self.speed_service_on:
                    self.get_logger().info("End of set speed duration, setting speeds to 0")
                self.speed_service_on = False
                self.x_vel_goal = 0
                self.y_vel_goal = 0
                self.theta_vel_goal = 0
            self.filter_speed_goals()
            wheel_speeds = self.ik_vel(
                self.x_vel_goal_filtered, self.y_vel_goal_filtered, self.theta_vel_goal_filtered)
            self.send_wheel_commands(wheel_speeds)
        elif self.mode is ZuuuModes.GOTO:
            x_vel, y_vel, theta_vel = 0, 0, 0
            if self.goto_service_on:
                distance = math.sqrt(
                    (self.x_goal - self.x_odom)**2 +
                    (self.y_goal - self.y_odom)**2
                )
                self.get_logger().info(
                    f"ON!! dist {distance}, ang {abs(angle_diff(self.theta_goal, self.theta_odom))}")
                if distance < self.xy_tol and abs(angle_diff(self.theta_goal, self.theta_odom)) < self.theta_tol:
                    self.goto_service_on = False
                else:
                    x_vel, y_vel, theta_vel = self.position_control()
                self.get_logger().info(
                    f"Sending x_vel {x_vel}, y_vel {y_vel}, theta_vel {theta_vel}")
                wheel_speeds = self.ik_vel(
                    x_vel, y_vel, theta_vel)
                self.send_wheel_commands(wheel_speeds)
            self.get_logger().info(
                f"self.goto_service_on {self.goto_service_on}")

        elif self.mode is ZuuuModes.EMERGENCY_STOP:
            self.get_logger().warning("Emergency stop requested")
            self.emergency_shutdown()

        else:
            self.get_logger().warning("unknown mode '{}', setting it to brake".format(self.mode))
            self.mode = ZuuuModes.BRAKE

        self.old_measure_timestamp = self.measure_timestamp
        self.measure_timestamp = self.get_clock().now()

        # Reading the measurements (this is what takes most of the time, ~9ms)
        self.read_measurements()
        self.update_wheel_speeds()

        if verbose:
            self.print_all_measurements()

        self.publish_wheel_speeds()
        self.tick_odom()
        if SAVE_CSV:
            self.csv_writer.writerow(
                [time.time() - self.t0, duty_cycles[0], (2*math.pi*self.omnibase.back_wheel_rpm/self.omnibase.half_poles)/60, self.vtheta])

        if verbose:
            self.get_logger().info("x_odom {}, y_odom {}, theta_odom {}".format(
                self.x_odom, self.y_odom, self.theta_odom))

        # Time measurement
        dt = time.time() - t
        if dt == 0:
            f = 0
        else:
            f = 1/dt
        if verbose:
            self.get_logger().info(
                "zuuu tick potential freq: {:.0f}Hz (dt={:.0f}ms)".format(f, 1000*dt))


def main(args=None):
    rclpy.init(args=args)
    node = ZuuuHAL()

    try:
        rclpy.spin(node)
    except Exception as e:
        traceback.print_exc()
    finally:
        node.emergency_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
