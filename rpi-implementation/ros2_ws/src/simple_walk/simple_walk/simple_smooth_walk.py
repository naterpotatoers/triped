import rclpy
from rclpy.node import Node

from mission_control_interfaces.msg import MissionControlToRobot
from servo_interfaces.msg import QuadrupedLegPositions

import math
from math import sqrt
import time


def smooth_step(t):
    if 0.0 <= t and t < 0.5:
        return 2.0 * t * t
    elif 0.5 <= t and t <= 1.0:
        return -2.0 * t * t + 4.0 * t - 1.0


def smooth_bump(t):
    # t <= 0: 0, t = 0.5: 1, t >= 1: 0
    if 0.0 <= t and t < 0.5:
        return smooth_step(2.0 * t)
    elif 0.5 <= t and t <= 1.0:
        return 1.0 - smooth_step(2.0 * t - 1.0)


def get_step_position(t, stride_size, raised_height, ratio_raised, raise_anticipation, stance_width, crossover_clearance, z_acc):
    t = t % (1.0 + ratio_raised)

    y_acc = 4.0 * stride_size / (ratio_raised * ratio_raised) * (1.0 + ratio_raised)

    x = 0.0
    y = 0.0
    z = 0.0

    # x
    if t < 1.0:
        x = -stance_width * 0.5
    elif t <= 1.0 + ratio_raised:
        x = -stance_width * 0.5 - max(0.0, (crossover_clearance - stance_width)) * smooth_bump(1.0 / (ratio_raised) * (t - 1.0))

    # y
    if t < 1.0:  # moving foot backward
        y = stride_size * (0.5 - t)
    elif t < 1.0 + ratio_raised * 0.5:  # moving foot forward, accelerating
        y = y_acc * 0.5 * t*t + (stride_size + y_acc) * (-t + 0.5)
    elif t < 1.0 + ratio_raised:  # moving foot forward, decelerating
        y = -y_acc * 0.5 * (-t + 2.0 + ratio_raised)**2 + (stride_size + y_acc) * (-t + 1.5 + ratio_raised)

    # z
    if t < raise_anticipation * ratio_raised - sqrt(4.0 / z_acc):
        z = 1.0
    elif t < raise_anticipation * ratio_raised:
        z = 1.0 - smooth_step(sqrt(z_acc * 0.25) * (t - raise_anticipation * ratio_raised) + 1.0)
    elif t < 1.0 - raise_anticipation * ratio_raised:
        z = 0.0
    elif t < 1.0 - raise_anticipation * ratio_raised + sqrt(4.0 / z_acc):
        z = smooth_step(sqrt(z_acc * 0.25) * (t - 1.0 + raise_anticipation * ratio_raised))
    elif t < 1.0 + ratio_raised * 0.5:
        z = 1.0
    elif t < 1.0 + ratio_raised * (1.0 + raise_anticipation) - sqrt(4.0 / z_acc):
        z = 1.0
    elif t < 1.0 + ratio_raised:
        z = 1.0 - smooth_step(sqrt(z_acc * 0.25) * (t - 1.0 - ratio_raised - raise_anticipation * ratio_raised) + 1.0)
    z *= raised_height

    return (x, y, z)


def wrap_direction_error(target_radians, current_radians):
    pi = math.pi
    pi2 = 2.0 * math.pi
    return ((target_radians - current_radians) % pi2 + pi) % pi2 - pi


class QuadSimpleWalk(Node):

    def __init__(self):
        super().__init__('quadruped_simple_smooth_walk')

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
        self.max_speed = 2
        self.speed = 0.0

        self.t = 0.0

        self.get_logger().info('simple smooth walk running')

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
        z = 9.0  # inches

        stride_size = 6.0
        raised_height = 2.0
        ratio_raised = 1.0
        stance_width_x = 10
        stance_width_y = 10.0
        raise_anticipation = 0.1
        z_acc = 13.0
        crossover_clearance = 1.0  # inches

        msg = QuadrupedLegPositions()

        leg_t = t
        (step_x, step_y, step_z) = get_step_position(leg_t, stride_size, raised_height, ratio_raised, raise_anticipation, stance_width_x, crossover_clearance, z_acc)
        msg.leg_front_right.x = -step_x
        msg.leg_front_right.y = stance_width_y + step_y
        msg.leg_front_right.z = -z + step_z

        leg_t = t - 0.5 * (1 + ratio_raised)
        (step_x, step_y, step_z) = get_step_position(leg_t, stride_size, raised_height, ratio_raised, raise_anticipation, stance_width_x, crossover_clearance, z_acc)
        msg.leg_front_left.x = step_x
        msg.leg_front_left.y = stance_width_y + step_y
        msg.leg_front_left.z = -z + step_z

        leg_t = t
        (step_x, step_y, step_z) = get_step_position(leg_t, stride_size, raised_height, ratio_raised, raise_anticipation, stance_width_x, crossover_clearance, z_acc)
        msg.leg_rear_left.x = step_x
        msg.leg_rear_left.y = -stance_width_y + step_y
        msg.leg_rear_left.z = -z + step_z

        leg_t = t - 0.5 * (1 + ratio_raised)
        (step_x, step_y, step_z) = get_step_position(leg_t, stride_size, raised_height, ratio_raised, raise_anticipation, stance_width_x, crossover_clearance, z_acc)
        msg.leg_rear_right.x = -step_x
        msg.leg_rear_right.y = -stance_width_y + step_y
        msg.leg_rear_right.z = -z + step_z

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
