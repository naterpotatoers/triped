import rclpy
from rclpy.node import Node

from servo_interfaces.msg import QuadrupedLegPositions

import math
import time


def get_step_position(t, stride_size, raised_height, ratio_raised):
    t = t % (1.0 + ratio_raised)
    if t <= 1.0:
        return (
            stride_size * (0.5 - t),
            0.0,
        )
    else:
        t -= 1
        return (
            stride_size * (-0.5 + t / ratio_raised),
            raised_height * math.sin(math.pi / ratio_raised * t),
        )


def get_shift_amount(t, ratio_raised, shift_amount):
    t = t % (1.0 + ratio_raised)
    if t <= 1.0:
        return shift_amount * (2.0 * t - 1)**2
    else:
        return shift_amount


class QuadSimpleWalk(Node):

    def __init__(self):
        super().__init__('quadruped_simple_walk')

        self.publisher_ = self.create_publisher(
            QuadrupedLegPositions,
            'leg_positions',
            10)
        update_period = 1 / 100  # seconds
        self.timer = self.create_timer(update_period, self.timer_callback)

        self.get_logger().info('simple walk running')
    
    def timer_callback(self):
        now = time.perf_counter()

        direction = -90
        dir_x, dir_y = (
            math.cos(direction + math.pi * 0.5),
            math.sin(direction + math.pi * 0.5),
        )

        t = now * 0.2
        z = -10.0  # inches

        stride_size = 4.0
        raised_height = 2.0
        ratio_raised = 0.2
        stance_width_x = 2
        stance_width_y = 3
        shift_amplitude = 0.8

        msg = QuadrupedLegPositions()

        shift_x = 0.0
        shift_y = 0.0

        leg_t = t
        (step_xy, step_z) = get_step_position(leg_t, stride_size, raised_height, ratio_raised)
        msg.leg_front_right.x = stance_width_x + dir_x * step_xy
        msg.leg_front_right.y = stance_width_y + dir_y * step_xy
        msg.leg_front_right.z = z + step_z
        shift_amount = get_shift_amount(leg_t, ratio_raised, shift_amplitude)
        shift_x += shift_amount * 2
        shift_y += shift_amount * 3

        leg_t = t - 0.5 * (1 + ratio_raised)
        (step_xy, step_z) = get_step_position(leg_t, stride_size, raised_height, ratio_raised)
        msg.leg_front_left.x = -stance_width_x + dir_x * step_xy
        msg.leg_front_left.y = stance_width_y + dir_y * step_xy
        msg.leg_front_left.z = z + step_z
        shift_amount = get_shift_amount(leg_t, ratio_raised, shift_amplitude)
        shift_x += shift_amount * -2
        shift_y += shift_amount * 3

        leg_t = t - 0.25 * (1 + ratio_raised)
        (step_xy, step_z) = get_step_position(leg_t, stride_size, raised_height, ratio_raised)
        msg.leg_rear_left.x = -stance_width_x + dir_x * step_xy
        msg.leg_rear_left.y = -stance_width_y + dir_y * step_xy
        msg.leg_rear_left.z = z + step_z
        shift_amount = get_shift_amount(leg_t, ratio_raised, shift_amplitude)
        shift_x += shift_amount * -2
        shift_y += shift_amount * -3

        leg_t = t - 0.75 * (1 + ratio_raised)
        (step_xy, step_z) = get_step_position(leg_t, stride_size, raised_height, ratio_raised)
        msg.leg_rear_right.x = stance_width_x + dir_x * step_xy
        msg.leg_rear_right.y = -stance_width_y + dir_y * step_xy
        msg.leg_rear_right.z = z + step_z
        shift_amount = get_shift_amount(leg_t, ratio_raised, shift_amplitude)
        shift_x += shift_amount * 2
        shift_y += shift_amount * -3

#        print('%.2f\t%.2f' % (shift_x, shift_y))
        
        msg.leg_front_right.x += shift_x
        msg.leg_front_right.y += shift_y
        msg.leg_front_left.x += shift_x
        msg.leg_front_left.y += shift_y
        msg.leg_rear_left.x += shift_x
        msg.leg_rear_left.y += shift_y
        msg.leg_rear_right.x += shift_x
        msg.leg_rear_right.y += shift_y

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    quad_simple_walk = QuadSimpleWalk()

    rclpy.spin(quad_simple_walk)

    quad_simple_walk.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
