import rclpy
from rclpy.node import Node

from mission_control_interfaces.msg import MissionControlToRobot
from servo_interfaces.msg import QuadrupedLegPositions

import math
import time


def smooth_bump(t):
    # t <= 0: 0, t = 0.5: 1, t >= 1: 0
    t = t * 2.0 - 1.0
    y = 0.0
    if -1.0 <= t < -0.5:
        y = 0.5 * t * t + t + 0.5
    elif -0.5 <= t < 0.5:
        y = -0.5 * t * t + 0.25
    elif 0.5 <= t <= 1.0:
        y = 0.5 * t * t - t + 0.5
    return 4.0 * y


def get_step_position(t, stride_size, raised_height, ratio_raised, crossover_clearance):
    t = t % (1.0 + ratio_raised)
    if t <= 1.0:
        x = 0.0
        y = stride_size * (0.5 - t)
        z = 0.0
    else:
        x = -crossover_clearance * smooth_bump(2.0 / (stride_size * ratio_raised) * (t - 1.0))
        y = stride_size * (-0.5 + (t - 1.0) / ratio_raised)
        z = raised_height * math.sin(math.pi / ratio_raised * (t - 1.0))
    return (x, y, z)


def wrap_direction_error(target_radians, current_radians):
    pi = math.pi
    pi2 = 2.0 * math.pi
    return ((target_radians - current_radians) % pi2 + pi) % pi2 - pi


class QuadSimpleWalk(Node):

    def __init__(self):
        super().__init__('quadruped_simple_narrow_walk')

        self.subscription_ = self.create_subscription(
            MissionControlToRobot,
            'mission_control_to_robot',
            self.mission_control_callback,
            10)
        self.publisher_ = self.create_publisher(
            QuadrupedLegPositions,
            'leg_positions',
            10)
        update_period = 1 / 100  # seconds
        self.timer = self.create_timer(update_period, self.timer_callback)
        self.last_timer_time = time.perf_counter()

        self.target_direction = 0.0
        self.direction = 0.0
        self.max_speed = 1.5
        self.speed = 0.0

        self.t = 0.0

        self.get_logger().info('simple walk running')

    def mission_control_callback(self, msg):
        self.speed = math.sqrt(msg.speed_forward**2 + msg.speed_right**2)
        if (self.speed > 0.01):
            self.target_direction = math.atan2(
                -msg.speed_right,
                msg.speed_forward)
        self.speed = max(0.0, min(1.0, self.speed))
    
    def timer_callback(self):
        now = time.perf_counter()

        t = self.t
        z = -10.0  # inches

        stride_size = 8.0
        raised_height = 2.0
        ratio_raised = 1.0
        stance_width_x = 2.5
        stance_width_y = 7.0
        crossover_clearance = 1.0  # inches

        msg = QuadrupedLegPositions()

        leg_t = t
        (step_x, step_y, step_z) = get_step_position(leg_t, stride_size, raised_height, ratio_raised, crossover_clearance)
        msg.leg_front_right.x = stance_width_x + step_x
        msg.leg_front_right.y = stance_width_y + step_y
        msg.leg_front_right.z = z + step_z

        leg_t = t - 0.5 * (1 + ratio_raised)
        (step_x, step_y, step_z) = get_step_position(leg_t, stride_size, raised_height, ratio_raised, crossover_clearance)
        msg.leg_front_left.x = -stance_width_x - step_x
        msg.leg_front_left.y = stance_width_y + step_y
        msg.leg_front_left.z = z + step_z

        leg_t = t
        (step_x, step_y, step_z) = get_step_position(leg_t, stride_size, raised_height, ratio_raised, crossover_clearance)
        msg.leg_rear_left.x = -stance_width_x - step_x
        msg.leg_rear_left.y = -stance_width_y + step_y
        msg.leg_rear_left.z = z + step_z

        leg_t = t - 0.5 * (1 + ratio_raised)
        (step_x, step_y, step_z) = get_step_position(leg_t, stride_size, raised_height, ratio_raised, crossover_clearance)
        msg.leg_rear_right.x = stance_width_x + step_x
        msg.leg_rear_right.y = -stance_width_y + step_y
        msg.leg_rear_right.z = z + step_z

        self.publisher_.publish(msg)

        # Advance "time" parameter according to desired speed
        self.speed = 1.0
        self.t += (now - self.last_timer_time) * self.speed * self.max_speed

        # Decay speed over time when not being updated by mission control
        # to mitigate connection-loss issues
        halt_time = 0.5  # seconds it takes for the robot to halt when not receiving commands
        self.speed = max(0.0, self.speed - self.max_speed * (now - self.last_timer_time) / halt_time)

        self.last_timer_time = now


def main(args=None):
    rclpy.init(args=args)

    quad_simple_walk = QuadSimpleWalk()

    rclpy.spin(quad_simple_walk)

    quad_simple_walk.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
