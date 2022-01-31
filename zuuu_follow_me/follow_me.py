import rclpy
from rclpy.node import Node
import pygame
import time
import numpy as np
import traceback
import sys
from pyvesc.VESC import MultiVESC


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

    def emergency_shutdown(self):
        self.get_logger().warn("Emergency shutdown!")
        self.omnibase.back_wheel.set_duty_cycle(0)
        self.omnibase.left_wheel.set_duty_cycle(0)
        self.omnibase.right_wheel.set_duty_cycle(0)
        sys.exit(1)

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
            self.tick_controller()
            if self.mode == CONTROLLER:
                duty_cycles = self.cycle_from_joystick()
                self.omnibase.back_wheel.set_duty_cycle(
                    duty_cycles[0])
                self.omnibase.left_wheel.set_duty_cycle(
                    duty_cycles[2])
                self.omnibase.right_wheel.set_duty_cycle(
                    duty_cycles[1])
            elif self.mode == FOLLOW_ME:
                pass
            else:
                self.get_logger().warn("Can't happen ffs")
                self.emergency_shutdown()


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
