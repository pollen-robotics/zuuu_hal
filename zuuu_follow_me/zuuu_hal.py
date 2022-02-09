import rclpy
from rclpy.node import Node
import time
import math
import numpy as np
import traceback
import sys
from pyvesc.VESC import MultiVESC
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile


"""
This node has the responsability to interact with zuuu's hardware and 'ROSify' the inputs and outputs.
Zuuu's hardware = 3 motor controllers. The LIDAR, camera and other sensors are NOT handled here.
Specificaly, this node will periodically read the selected measurements from the controllers of the wheels (speed, temperature, IMUs, etc) and publish them into the adequate topics (odom, etc).
This node will also subscribe to /cmd_vel and write commands to the 3 motor controllers.
"""

class MobileBase:
    def __init__(
        self,
        serial_port='/dev/ttyACM0',
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

    def read_all_measurements(self):
        self.left_wheel_measurements = self.left_wheel.get_measurements()
        self.right_wheel_measurements = self.right_wheel.get_measurements()
        self.back_wheel_measurements = self.back_wheel.get_measurements()


class ZuuuHAL(Node):
    def __init__(self):
        super().__init__('zuuu_hal')
        self.get_logger().info("Starting zuuu_hal!")

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription  # prevent unused variable warning... JESUS WHAT HAVE WE BECOME

        self.omnibase = MobileBase()
        self.max_duty_cyle = 0.2 # max is 1
        self.cmd_vel_timeout = 0.2
        self.cmd_vel = None
        # *sigh* if needed use: https://github.com/ros2/rclpy/blob/master/rclpy/test/test_time.py
        self.cmd_vel_t0 = time.time()
        self.get_logger().info(
            "zuuu_hal started, you can write to cmd_vel to move the robot")
        self.get_logger().info("List of published topics: TODO")

        self.create_timer(0.01, self.main_tick)

    def emergency_shutdown(self):
        self.omnibase.back_wheel.set_duty_cycle(0)
        self.omnibase.left_wheel.set_duty_cycle(0)
        self.omnibase.right_wheel.set_duty_cycle(0)
        self.get_logger().warn("Emergency shutdown!")
        sys.exit(1)

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg
        self.cmd_vel_t0 = time.time()

    def angle_diff(self, a, b):
        # Returns the smallest distance between 2 angles
        d = a - b
        d = ((d + math.pi) % (2 * math.pi)) - math.pi
        return d


    def ik_vel(self, x, y, rot):
        """Takes 2 linear speeds and 1 rot speed (robot's egocentric frame) and outputs the PWM to apply to each of the 3 motors in an omni setup 

        Args:
            x (float): x speed (between 0 and 1). Positive "in front" of the robot.
            y (float): y speed (between 0 and 1). Positive "to the left" of the robot.
            rot (float): rotational speed (between 0 and 1). Positive counter-clock wise.
        """

        cycle_back = -y + rot
        cycle_right = (-y*np.cos(120*np.pi/180)) + \
            (x*np.sin(120*np.pi/180)) + rot
        cycle_left = (-y*np.cos(240*np.pi/180)) + \
            (x*np.sin(240*np.pi/180)) + rot

        return [cycle_back, cycle_right, cycle_left]

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
        self.get_logger().info("{}".format(to_print))

    def limit_duty_cycles(self, duty_cycles):
        # Between +- max_duty_cyle
        for i in range(len(duty_cycles)) :
            if duty_cycles[i] < 0 :
                 duty_cycles[i] = max(-self.max_duty_cyle, duty_cycles[i])
            else :
                 duty_cycles[i] = min(self.max_duty_cyle, duty_cycles[i])
        return duty_cycles


    def main_tick(self, verbose=False):
        duty_cycles = [0, 0, 0]
        t = time.time()

        # If too much time without an order, the speeds of he wheels are set to 0 for safety.
        if (self.cmd_vel is not None) and ((t - self.cmd_vel_t0) < self.cmd_vel_timeout) :
            x = self.cmd_vel.linear.x
            y = self.cmd_vel.linear.y
            theta = self.cmd_vel.angular.z
            duty_cycles = self.ik_vel(x, y, theta)
        duty_cycles = self.limit_duty_cycles(duty_cycles)
            
        # Actually sending the commands
        if verbose :
            self.get_logger().info("cycles : {}".format(duty_cycles))
        self.get_logger().info("cycles : {}".format(duty_cycles))
        self.omnibase.back_wheel.set_duty_cycle(
            duty_cycles[0])
        self.omnibase.left_wheel.set_duty_cycle(
            duty_cycles[2])
        self.omnibase.right_wheel.set_duty_cycle(
            duty_cycles[1])

        # Reading the measurements (this is what takes most of the time, ~9ms)
        self.omnibase.read_all_measurements()
        if verbose :
            self.print_all_measurements()
        
        # Time measurement
        dt = time.time() - t
        if dt == 0:
            f = 0
        else:
            f = 1/dt
        if verbose :
            self.get_logger().info("zuuu tick potential freq: {:.0f}Hz (dt={:.0f}ms)".format(f, 1000*dt))


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

