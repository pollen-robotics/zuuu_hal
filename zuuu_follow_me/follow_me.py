import rclpy
from rclpy.node import Node
import pygame
import time
import math
import numpy as np
import traceback
import sys
from pyvesc.VESC import MultiVESC
from sensor_msgs.msg import LaserScan
import cv2


# Button  0 = X
# Button  1 = O
# Button  2 = Triangle
# Button  3 = Carre
# Button  4 = l1
# Button  5 = r1
# Button  6 = l2
# Button  7 = r2
# Button  8 = share
# Button  9 = options
# Button 10 = ps_button
# Button 11 = joy_left
# Button 12 = joy_right

CONTROLLER = 0
FOLLOW_ME = 1


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


class FollowMe(Node):
    def __init__(self):
        super().__init__('follow_me')
        self.get_logger().info("Starting follow me behavior!")

        self.laser_scan = None
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            1)

        # we could tick pygame with a callback:
        # self.create_timer(0.2, self.timer_callback)
        pygame.init()
        pygame.joystick.init()

        self.mode = CONTROLLER
        self.nb_joy = pygame.joystick.get_count()
        if self.nb_joy < 1:
            self.get_logger().error("No controller detected.")
            self.emergency_shutdown()
        self.get_logger().info("nb joysticks: {}".format(self.nb_joy))
        self.j = pygame.joystick.Joystick(0)
        self.omnibase = MobileBase()
        self.lin_speed_ratio = 0.2
        self.rot_speed_ratio = 0.1
        self.loop_freq = 100
        self.clock = pygame.time.Clock()
        self.t0 = time.time()

    def emergency_shutdown(self):
        self.get_logger().warn("Emergency shutdown!")
        self.omnibase.back_wheel.set_duty_cycle(0)
        self.omnibase.left_wheel.set_duty_cycle(0)
        self.omnibase.right_wheel.set_duty_cycle(0)
        sys.exit(1)

    def scan_callback(self, msg):
        self.laser_scan = msg

    def angle_diff(a, b):
        # Returns the smallest distance between 2 angles
        d = a - b
        d = ((d + math.pi) % (2 * math.pi)) - math.pi
        return d

    def get_barycenter_offset(self, range_min, range_max, detection_angle, verbose=True):
        """Listen to the /scan LaserScan topic and outputs dlin_percent and dang_percent which represent how far away from the center of the chosen shape the barycenter of points is.
        """
        # frame_id: "base_scan"
        # angle_min: 0.0
        # angle_max: 6.2657318115234375
        # angle_increment: 0.01745329238474369
        # time_increment: 2.9880000511184335e-05
        # scan_time: 0.0
        # range_min: 0.11999999731779099
        # range_max: 3.5
        if self.laser_scan is None:
            return 0, 0
        absolute_range_max = 3.5
        absolute_angle_max = 6.2657318115234375
        half_angle = detection_angle/2
        # rospy.loginfo("Waiting for laser scan")
        # laser_scan = rospy.wait_for_message("/tb3/scan", LaserScan, timeout=None)
        # rospy.loginfo("laser scan received!")

        angle_increment = self.laser_scan.angle_increment
        pixel_per_meter = 250
        image_size = int(absolute_range_max * pixel_per_meter)
        height = image_size
        width = image_size
        self.get_logger().info("Image will be {}x{}".format(width, height))
        image = np.zeros((height, width, 3), np.uint8)
        index = -1
        center_x = width / 2
        center_y = 0.5*(range_max + range_min) * pixel_per_meter
        sum_x = 0
        sum_y = 0

        nb_points = 0
        if verbose:
            for i in range(width):
                for j in range(height):
                    angle = math.atan2(i-(width/2), j)
                    dist = math.sqrt(((width/2)-i)**2 + j**2)/pixel_per_meter
                    if dist >= range_min and dist <= range_max and abs(self.angle_diff(angle, 0)) < half_angle:
                        image[j, i] = (50, 50, 100)  # y, x as always

        # the 0, 0 point of the laserscan will be drawn on the point x = width/2 and y = 0 (so the top of the image)
        for r in self.laser_scan.ranges:
            index += 1
            d = 0
            try:
                d = float(r)
                if np.isnan(d):
                    continue
            except:
                # inf?
                continue
            if d < range_min or d >= range_max:
                continue
            angle = self.laser_scan.angle_min + index * angle_increment
            if abs(self.angle_diff(angle, 0)) > half_angle:
                continue
            x = int(round(width / 2 + d * pixel_per_meter * math.sin(angle)))
            y = int(round(d * pixel_per_meter * math.cos(angle)))
            sum_x += x
            sum_y += y
            nb_points += 1
            if x >= 0 and x < width and y >= 0 and y < height:
                image[y, x] = (255, 255, 255)  # y, x as always
            # print("Adding point {}".format(round(d*math.cos(angle)), round(width/2 + d*math.sin(angle))))
        if nb_points > 0:
            sum_x = sum_x/nb_points
            sum_y = sum_y/nb_points
            image[int(sum_y), int(sum_x)] = (255, 0, 0)  # y, x as always
            # Careful, x in image frame is -y lidar frame, and y in image frame is x in lidar frame
            avg_angle = math.atan2(sum_x - width/2, sum_y)
            dist = math.sqrt((sum_x-center_x)**2 + (sum_y-center_y)**2)
            if center_y < sum_y:
                dist *= -1
            dlin_percent = max(-1, min(1, 2*dist /
                               (pixel_per_meter*(range_max - range_min))))
            dang_percent = max(-1, min(1, avg_angle/half_angle))
            self.get_logger().info("avg_x_pixels={}, avg_y_pixels){}, avg_angle={}, dist_pixels={}, dlin_percent={}, dang_percent={}, center_y={}".format(
                sum_x, sum_y, avg_angle, dist, dlin_percent, dang_percent, center_y))
        else:
            dlin_percent = 0
            dang_percent = 0

        if verbose:
            cv2.imshow("image", image)
            # cv2.imwrite("raw" + str(datetime.utcnow())+".png", image)
            cv2.waitKey(1)
        return dlin_percent, dang_percent

    def tick_controller(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.emergency_shutdown()
            elif event.type == pygame.JOYBUTTONDOWN:
                if self.j.get_button(2):
                    # intesity min, max, and duration in ms
                    self.j.rumble(1, 1, 200)
                    if self.mode == CONTROLLER:
                        self.mode = FOLLOW_ME
                        self.get_logger().info("Changing mode CONTROLLER -> FOLLOW_ME")
                    elif self.mode == FOLLOW_ME:
                        self.mode = CONTROLLER
                        self.get_logger().info("Changing mode FOLLOW_ME -> CONTROLLER")
                    else:
                        self.get_logger().warn("Can't happen ffs")
                        self.emergency_shutdown()

                if self.j.get_button(1):
                    self.get_logger().warn("Pressed emergency stop!")
                    self.emergency_shutdown()
            elif event.type == pygame.JOYBUTTONUP:
                pass

        if self.nb_joy != pygame.joystick.get_count():
            self.get_logger().warn("Controller disconnected!")
            self.emergency_shutdown()

    def rumble(self, duration):
        self.rumble_start = time.time()
        self.is_rumble = True
        self.rumble_duration = duration
        # Duration doesn't work, have to do it ourselves
        self.j.rumble(1, 1, 1000)

    def print_controller(self):
        # Get the name from the OS for the controller/joystick.
        name = self.j.get_name()
        self.get_logger().info("Joystick name: {}".format(name))

        # Usually axis run in pairs, up/down for one, and left/right for
        # the other.
        axes = self.j.get_numaxes()
        self.get_logger().info("Number of axes: {}".format(axes))

        for i in range(axes):
            axis = self.j.get_axis(i)
            self.get_logger().info("Axis {} value: {:>6.3f}".format(i, axis))

        buttons = self.j.get_numbuttons()
        self.get_logger().info("Number of buttons: {}".format(buttons))

        for i in range(buttons):
            button = self.j.get_button(i)
            self.get_logger().info("Button {:>2} value: {}".format(i, button))

    def cycle_from_joystick(self):
        cycle_max_t = self.lin_speed_ratio  # 0.2*factor
        cycle_max_r = self.rot_speed_ratio  # 0.1*factor

        y = -self.j.get_axis(1) * cycle_max_t
        x = self.j.get_axis(0) * cycle_max_t
        rot = -self.j.get_axis(3) * cycle_max_r

        return self.ik_vel(x, y, rot)

    def ik_vel(self, x, y, rot):
        """Takes 2 linear speeds and 1 rot speed and outputs the PWM to apply to each of the 3 motors in an omni setup 

        Args:
            x (float): x speed (between 0 and 1)
            y (float): y speed (between 0 and 1)
            rot (float): rotational speed (between 0 and 1)
        """

        cycle_back = x + rot
        cycle_right = (x*np.cos(120*np.pi/180)) + \
            (y*np.sin(120*np.pi/180)) + rot
        cycle_left = (x*np.cos(240*np.pi/180)) + \
            (y*np.sin(240*np.pi/180)) + rot

        return (cycle_back), (cycle_right), (cycle_left)

    def main(self):
        self.get_logger().info(
            "Started. Press O for emergency stop. Press triangle to change modes.")
        while True:
            t = time.time()
            dt = t - self.t0
            if dt == 0:
                f = 0
            else:
                f = 1/dt
            self.get_logger().info("Potential freq: {:.0f}Hz".format(f))
            # Caping the actual freq
            self.clock.tick(self.loop_freq)
            self.t0 = time.time()

            self.tick_controller()
            if self.mode == CONTROLLER:
                duty_cycles = self.cycle_from_joystick()

            elif self.mode == FOLLOW_ME:
                dlin_percent, dang_percent = self.get_barycenter_offset(
                    0.3, 0.7, math.pi/5)
                duty_cycles = self.ik_vel(dlin_percent, 0, dang_percent)
            else:
                self.get_logger().warn("Can't happen ffs")
                self.emergency_shutdown()
            # Actually sending the commands
            # self.omnibase.back_wheel.set_duty_cycle(
            #     duty_cycles[0])
            # self.omnibase.left_wheel.set_duty_cycle(
            #     duty_cycles[2])
            # self.omnibase.right_wheel.set_duty_cycle(
            #     duty_cycles[1])


def main(args=None):
    rclpy.init(args=args)
    node = FollowMe()

    try:
        node.main()
        rclpy.spin(node)
    except Exception as e:
        traceback.print_exc()
    finally:
        node.emergency_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
