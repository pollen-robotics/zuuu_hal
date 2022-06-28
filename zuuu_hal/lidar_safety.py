import math
import copy

from zuuu_hal.utils import angle_diff
from sensor_msgs.msg import LaserScan


"""
TODO
[rplidar_scan_publisher-1] [WARN] [1656351979.817559703] [rplidar_scan_publisher]: 
New subscription discovered on this topic, requesting incompatible QoS. 
No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
"""


class LidarSafety:
    def __init__(self, safety_distance, critical_distance, robot_collision_radius, speed_reduction_factor, logger):
        """Utility class to reduce Zuuu's speed when too close to obstacles seen by the LIDAR.
        Functional behaviour:
        - safety_distance >= critical_distance
        - Zuuu's speed is slowed down if the direction of speed matches the direction of at least 1 LIDAR point in the safety_distance range
        - Zuuu's speed is changed to 0 if the direction of speed matches the direction of at least 1 LIDAR point in the critical_distance range
        - If at least 1 point is in the critical distance, then even motions that move away from the obstacles are slowed down to the "safety_zone" speed

        """
        self.safety_distance = safety_distance
        self.critical_distance = critical_distance
        self.robot_collision_radius = robot_collision_radius
        self.speed_reduction_factor = speed_reduction_factor
        # List of [center_angle, angle_width]. e.g. the forbidden angle is center_angle +-angle_width
        self.unsafe_angles = []
        self.critical_angles = []
        self.at_least_one_critical = False
        self.logger = logger

    def clear_measures(self):
        self.unsafe_angles = []
        self.critical_angles = []
        self.at_least_one_critical = False

    def process_scan(self, msg):
        self.clear_measures()
        critical_scan = LaserScan()
        critical_scan.header = copy.deepcopy(msg.header)
        critical_scan.angle_min = msg.angle_min
        critical_scan.angle_max = msg.angle_max
        critical_scan.angle_increment = msg.angle_increment
        critical_scan.time_increment = msg.time_increment
        critical_scan.scan_time = msg.scan_time
        critical_scan.range_min = msg.range_min
        critical_scan.range_max = msg.range_max
        ranges = []
        intensities = []
        nb_critical=0
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i*msg.angle_increment
            ranges.append(0.0)
            intensities.append(0.0)
            if r < 0.01:
                # Code value for "no detection". e.g. the lidar filter that filters self collisions
                # Adding an unsafe angle to avoid going fast where we're blind (TODO on new Reachy Mobile)
                self.unsafe_angles.append(self.create_forbidden_angles(angle, 0.25))
                continue
            dist = self.dist_to_point(r, angle)
            if dist < self.critical_distance and (msg.intensities[i] > 0.1):
                self.at_least_one_critical = True
                self.critical_angles.append(
                    self.create_forbidden_angles(angle, dist))
                ranges[-1] = r
                intensities[-1] = msg.intensities[i]
                nb_critical+=1
                # self.logger.info(f"$$$ critacal angle:{angle}")
            elif dist < self.safety_distance:
                self.unsafe_angles.append(
                    self.create_forbidden_angles(angle, dist))
        critical_scan.ranges = ranges
        critical_scan.intensities = intensities
        # self.logger.info(f"############ critical points:{nb_critical}")

        return critical_scan

    def safety_check_speed_command(self, x_vel, y_vel, theta_vel):
        if len(self.unsafe_angles) == 0 and len(self.critical_angles) == 0:
            # self.logger.info(
            #     "There are no close obstacles, the speed commands are left untouched")
            return x_vel, y_vel, theta_vel
        elif len(self.critical_angles) > 0:
            # self.logger.info("Zuuu is very close to an obstacle.")
            # self.logger.info(f"len(self.critical_angles):{len(self.critical_angles):.1f}")
            if x_vel == 0.0 and y_vel == 0.0:
                # self.logger.info("A pure rotation is OK but still slowed down")
                return 0.0, 0.0, theta_vel*self.speed_reduction_factor
            direction = math.atan2(y_vel, x_vel)
            # self.logger.info(f"direction:{direction:.1f}")
            for pair in self.critical_angles:
                if abs(angle_diff(pair[0], direction)) < pair[1]:
                    # self.logger.info(
                    #     "If the direction matches a critical angle, the speed is 0 in x and y")
                    # self.logger.info(f"direction:{direction:.1f}, pair[0]:{pair[0]:.1f}, angle_diff:{angle_diff(pair[0], direction):.1f}, beta:{pair[1]:.1f}")
                    return 0.0, 0.0, theta_vel*self.speed_reduction_factor
            # self.logger.info(
            #     "The direction does not match a critical angle but the speed is still limited")
            return x_vel*self.speed_reduction_factor, y_vel*self.speed_reduction_factor, theta_vel*self.speed_reduction_factor
        else:
            # self.logger.info("Zuuu is moderately close to an obstacle.")
            if x_vel == 0.0 and y_vel == 0.0:
                # self.logger.info("A pure rotation is OK")
                return 0.0, 0.0, theta_vel
            direction = math.atan2(y_vel, x_vel)
            for pair in self.unsafe_angles:
                if abs(angle_diff(pair[0], direction)) < pair[1]:
                    # self.logger.info(
                    #     "If the direction matches an unsafe angle, the speed is reduced")
                    return x_vel*self.speed_reduction_factor, y_vel*self.speed_reduction_factor, theta_vel
            # self.logger.info(
            #     "The direction does not match an unsafe angle, the speed commands are left untouched")
            return x_vel, y_vel, theta_vel

    def dist_to_point(self, r, angle):
        x = r*math.cos(angle)
        y = r*math.sin(angle)
        # Not using the TF transforms because this is faster
        # TODO use a static TF2 transform instead
        x = x + 0.155
        dist = math.sqrt(x**2 + y**2)
        return dist

    def create_forbidden_angles(self, angle, dist):
        # Half of the forbidden angle span
        beta = abs(math.atan2(self.robot_collision_radius, dist))
        return [angle, beta]
