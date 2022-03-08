import rclpy
from rclpy.node import Node
import time
import math
import numpy as np
import traceback
import sys
from pyvesc.VESC import MultiVESC
from example_interfaces.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.constants import S_TO_NS
from collections import deque
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
# sudo apt install ros-foxy-tf-transformations
# sudo pip3 install transforms3d
# q = tf_transformations.quaternion_from_euler(r, p, y)
# r, p, y = tf_transformations.euler_from_quaternion(quaternion)


"""
This node has the responsability to interact with zuuu's hardware and 'ROSify' the inputs and outputs.
Zuuu's hardware = 3 motor controllers. The LIDAR, camera and other sensors are NOT handled here.
Specificaly, this node will periodically read the selected measurements from the controllers of the wheels (speed, temperature, IMUs, etc) and publish them into the adequate topics (odom, etc).
This node will also subscribe to /cmd_vel and write commands to the 3 motor controllers.
"""

# Interesting stuff can be found in Modern Robotics' chapters:
# 13.2 Omnidirectional Wheeled Mobile Robots
# 13.4 Odometry
# /!\ Our robot frame is different. Matching between their names (left) and ours (right): xb=y, yb-x, u1=uB, u2=uL, u3=uR


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
        self.left_wheel_nones, self.right_wheel_nones, self.back_wheel_nones = 0, 0, 0
        self.wheel_radius = 0.21/2.0
        self.wheel_to_center = 0.19588  # 0.188
        self.half_poles = 15.0
        self.left_wheel_rpm, self.right_wheel_rpm, self.back_wheel_rpm = 0, 0, 0
        self.left_wheel_avg_rpm, self.right_wheel_avg_rpm, self.back_wheel_avg_rpm = 0, 0, 0
        self.left_wheel_rpm_deque, self.right_wheel_rpm_deque, self.back_wheel_rpm_deque = deque(
            [], 10), deque([], 10), deque([], 10)

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
        self.get_logger().info("Starting zuuu_hal!")

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription  # prevent unused variable warning... JESUS WHAT HAVE WE BECOME
        self.pub_back_wheel_rpm = self.create_publisher(
            Float32, 'back_wheel_rpm', 2)
        self.pub_left_wheel_rpm = self.create_publisher(
            Float32, 'left_wheel_rpm', 2)
        self.pub_right_wheel_rpm = self.create_publisher(
            Float32, 'right_wheel_rpm', 2)

        self.pub_odom = self.create_publisher(
            Odometry, 'odom', 2)
        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)

        self.omnibase = MobileBase()
        self.max_duty_cyle = 0.2  # max is 1
        self.cmd_vel_timeout = 0.2
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
        self.old_measure_timestamp = self.get_clock().now()
        self.measure_timestamp = self.get_clock().now()
        # *sigh* if needed use: https://github.com/ros2/rclpy/blob/master/rclpy/test/test_time.py
        self.cmd_vel_t0 = time.time()
        self.get_logger().info(
            "zuuu_hal started, you can write to cmd_vel to move the robot")
        self.get_logger().info("List of published topics: TODO")

        self.create_timer(0.012, self.main_tick)

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

    def dk_vel(self, rot_l, rot_r, rot_b):
        """Takes the 3 rotational speeds (in rpm) of the 3 wheels and outputs the x linear speed, y, linear speed and rotational speed
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
        to_print += "\n\n Nones left:{}, right:{}, back:{}".format(
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

        # TODO tune these numbers
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
        # Local speeds in egocentric frame. Care, "rpm" are actually erpm and need to be devided by half the amount of magnetic poles to get the actual rpm.
        self.x_vel, self.y_vel, self.theta_vel = self.dk_vel(self.omnibase.left_wheel_rpm/self.omnibase.half_poles,
                                                             self.omnibase.right_wheel_rpm/self.omnibase.half_poles, self.omnibase.back_wheel_rpm/self.omnibase.half_poles)
        self.get_logger().info(
            "IK vel : {:.2f}, {:.2f}, {:.2f}".format(self.x_vel, self.y_vel, self.theta_vel))
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

        self.publish_odometry_and_tf()

    def limit_duty_cycles(self, duty_cycles):
        # Between +- max_duty_cyle
        for i in range(len(duty_cycles)):
            if duty_cycles[i] < 0:
                duty_cycles[i] = max(-self.max_duty_cyle, duty_cycles[i])
            else:
                duty_cycles[i] = min(self.max_duty_cyle, duty_cycles[i])
        return duty_cycles

    def main_tick(self, verbose=True):
        duty_cycles = [0, 0, 0]
        t = time.time()

        # If too much time without an order, the speeds of he wheels are set to 0 for safety.
        if (self.cmd_vel is not None) and ((t - self.cmd_vel_t0) < self.cmd_vel_timeout):
            x = self.cmd_vel.linear.x
            y = self.cmd_vel.linear.y
            theta = self.cmd_vel.angular.z
            duty_cycles = self.ik_vel(x, y, theta)
        duty_cycles = self.limit_duty_cycles(duty_cycles)

        # Actually sending the commands
        if verbose:
            self.get_logger().info("cycles : {}".format(duty_cycles))
        self.omnibase.back_wheel.set_duty_cycle(
            duty_cycles[0])
        self.omnibase.left_wheel.set_duty_cycle(
            duty_cycles[2])
        self.omnibase.right_wheel.set_duty_cycle(
            duty_cycles[1])

        self.old_measure_timestamp = self.measure_timestamp
        self.measure_timestamp = self.get_clock().now()

        # Reading the measurements (this is what takes most of the time, ~9ms)
        self.omnibase.read_all_measurements()
        self.update_wheel_speeds()

        if verbose:
            self.print_all_measurements()

        self.publish_wheel_speeds()
        self.tick_odom()

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
