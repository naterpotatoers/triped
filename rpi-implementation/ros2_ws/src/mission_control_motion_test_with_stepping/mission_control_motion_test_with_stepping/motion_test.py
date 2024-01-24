import rclpy
from rclpy.node import Node

from mission_control_interfaces.msg import MissionControlToRobot
from servo_interfaces.msg import QuadrupedLegPositions
from sensor_msgs.msg import Joy

import math
import time



class QuadMCMotionTest(Node):

    def __init__(self):
        super().__init__('motion_test')

        self.joy_subscription_ = self.create_subscription(
            MissionControlToRobot,
            'mission_control_to_robot',
            self.mission_control_callback,
            10)
        self.subscription_ = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        self.publisher_ = self.create_publisher(
            QuadrupedLegPositions,
            'leg_positions',
            10)
        
        self.leg_front_right_x = 7.0
        self.leg_front_right_y = 7.0
        self.leg_front_right_z = 0.0
        self.leg_front_left_x = -7.0
        self.leg_front_left_y = 7.0
        self.leg_front_left_z = 0.0
        self.leg_rear_left_x = -7.0
        self.leg_rear_left_y = -7.0
        self.leg_rear_left_z = 0.0
        self.leg_rear_right_x = 7.0
        self.leg_rear_right_y = -7.0
        self.leg_rear_right_z = 0.0
        
        self.x_vel = 0.0
        self.y_vel = 0.0
        self.z = 8.5
        self.target_z = 8.5
        self.turn_vel = 0.0
        self.turn_theta = 0.0
        self.tilt_vel = 0.0
        self.tilt_theta = 0.0
        self.leg_raised_vel_x = 0.0
        self.leg_raised_vel_y = 0.0

        self.xbox_button_a_last_pressed = False
        self.xbox_button_b_last_pressed = False
        self.xbox_button_x_last_pressed = False
        self.xbox_button_y_last_pressed = False

        self.leg_front_right_raised = False
        self.leg_front_left_raised = False
        self.leg_rear_left_raised = False
        self.leg_rear_right_raised = False

        self.radius = 2.0  # inches, radius of leaning
        self.max_turn_speed = 45 * math.pi / 180  # radians per second
        self.max_tilt_speed = 45 * math.pi / 180  # radians per second
        self.max_tilt_theta = 30 * math.pi / 180  # radians
        self.leg_raise_height = 1.5  # inches
        self.leg_raise_speed = 2.0  # inches per second
        self.movement_speed = 6.0  # inches per second
        
        update_period = 1 / 100  # seconds
        self.timer = self.create_timer(update_period, self.timer_callback)
        self.last_update_time = time.perf_counter()

        self.get_logger().info('mission control motion test running')
    
    def mission_control_callback(self, mc_msg):
        any_leg_raised = self.leg_front_right_raised or \
                         self.leg_front_left_raised or \
                         self.leg_rear_left_raised or \
                         self.leg_rear_right_raised

        self.x_vel = mc_msg.speed_right
        self.y_vel = mc_msg.speed_forward
        self.target_z = 8.5 + 2.5 * mc_msg.height  # inches

        if not any_leg_raised:
            self.turn_vel = mc_msg.speed_angular
            self.tilt_vel = mc_msg.tilt
            self.leg_raised_vel_x = 0.0
            self.leg_raised_vel_y = 0.0
        else:
            self.turn_vel = 0.0
            self.tilt_vel = 0.0
            self.leg_raised_vel_x = mc_msg.speed_angular
            self.leg_raised_vel_y = mc_msg.tilt

        # self.get_logger().info('target < x, y, z, turn, tilt > = < {:5.2f}, {:5.2f}, {:5.2f}, {:5.2f}, {:5.2f} >'.format(
        #     self.target_x, self.target_y, self.target_z, self.target_turn_theta, self.target_tilt_theta))

    def joy_callback(self, joy_msg):
        xbox_button_a_pressed = joy_msg.buttons[0] > 0
        xbox_button_b_pressed = joy_msg.buttons[1] > 0
        xbox_button_x_pressed = joy_msg.buttons[3] > 0
        xbox_button_y_pressed = joy_msg.buttons[4] > 0

        btn_a = xbox_button_a_pressed and not self.xbox_button_a_last_pressed
        btn_b = xbox_button_b_pressed and not self.xbox_button_b_last_pressed
        btn_x = xbox_button_x_pressed and not self.xbox_button_x_last_pressed
        btn_y = xbox_button_y_pressed and not self.xbox_button_y_last_pressed

        any_leg_raised = self.leg_front_right_raised or \
                         self.leg_front_left_raised or \
                         self.leg_rear_left_raised or \
                         self.leg_rear_right_raised

        num_buttons_pressed = btn_a + btn_b + btn_x + btn_y
        if num_buttons_pressed > 1:
            self.leg_front_right_raised = False
            self.leg_front_left_raised = False
            self.leg_rear_left_raised = False
            self.leg_rear_right_raised = False
        elif num_buttons_pressed == 1:
            if any_leg_raised:
                self.leg_front_right_raised = False
                self.leg_front_left_raised = False
                self.leg_rear_left_raised = False
                self.leg_rear_right_raised = False
            else:
                if btn_y:
                    self.leg_front_right_raised = True
                if btn_x:
                    self.leg_front_left_raised = True
                if btn_a:
                    self.leg_rear_left_raised = True
                if btn_b:
                    self.leg_rear_right_raised = True

        self.xbox_button_a_last_pressed = xbox_button_a_pressed
        self.xbox_button_b_last_pressed = xbox_button_b_pressed
        self.xbox_button_x_last_pressed = xbox_button_x_pressed
        self.xbox_button_y_last_pressed = xbox_button_y_pressed

    def timer_callback(self):
        now = time.perf_counter()
        time_elapsed = now - self.last_update_time

        # EMA of controller input
        self.z += (self.target_z - self.z) * min(1.0, time_elapsed / 0.1)
        self.turn_theta += self.turn_vel * self.max_turn_speed * time_elapsed
        self.tilt_theta += self.tilt_vel * self.max_tilt_speed * time_elapsed
        self.tilt_theta = max(-self.max_tilt_theta, min(self.max_tilt_theta, self.tilt_theta))

        # self.get_logger().info('< x, y, z, turn, tilt > = < {:5.2f}, {:5.2f}, {:5.2f}, {:5.2f}, {:5.2f} >'.format(
        #     self.x, self.y, self.z, self.turn_theta, self.tilt_theta))

        x_vel = self.x_vel * math.cos(-self.turn_theta) - self.y_vel * math.sin(-self.turn_theta)
        y_vel = self.x_vel * math.sin(-self.turn_theta) + self.y_vel * math.cos(-self.turn_theta)

        self.leg_front_right_x -= x_vel * self.movement_speed * time_elapsed
        self.leg_front_right_y -= y_vel * self.movement_speed * time_elapsed
        self.leg_front_left_x -= x_vel * self.movement_speed * time_elapsed
        self.leg_front_left_y -= y_vel * self.movement_speed * time_elapsed
        self.leg_rear_left_x -= x_vel * self.movement_speed * time_elapsed
        self.leg_rear_left_y -= y_vel * self.movement_speed * time_elapsed
        self.leg_rear_right_x -= x_vel * self.movement_speed * time_elapsed
        self.leg_rear_right_y -= y_vel * self.movement_speed * time_elapsed

        leg_raised_vel_x = self.leg_raised_vel_x * math.cos(self.turn_theta) - self.leg_raised_vel_y * math.sin(self.turn_theta)
        leg_raised_vel_y = self.leg_raised_vel_y * math.sin(self.turn_theta) + self.leg_raised_vel_y * math.cos(self.turn_theta)

        # Raise/lower legs
        if self.leg_front_right_raised:
            self.leg_front_right_x += leg_raised_vel_x * self.movement_speed * time_elapsed
            self.leg_front_right_y += leg_raised_vel_y * self.movement_speed * time_elapsed
            self.leg_front_right_z += self.leg_raise_speed * time_elapsed
        else:
            self.leg_front_right_z -= self.leg_raise_speed * time_elapsed
        self.leg_front_right_z = max(0.0, min(self.leg_raise_height, self.leg_front_right_z))

        if self.leg_front_left_raised:
            self.leg_front_left_x += leg_raised_vel_x * self.movement_speed * time_elapsed
            self.leg_front_left_y += leg_raised_vel_y * self.movement_speed * time_elapsed
            self.leg_front_left_z += self.leg_raise_speed * time_elapsed
        else:
            self.leg_front_left_z -= self.leg_raise_speed * time_elapsed
        self.leg_front_left_z = max(0.0, min(self.leg_raise_height, self.leg_front_left_z))

        if self.leg_rear_left_raised:
            self.leg_rear_left_x += leg_raised_vel_x * self.movement_speed * time_elapsed
            self.leg_rear_left_y += leg_raised_vel_y * self.movement_speed * time_elapsed
            self.leg_rear_left_z += self.leg_raise_speed * time_elapsed
        else:
            self.leg_rear_left_z -= self.leg_raise_speed * time_elapsed
        self.leg_rear_left_z = max(0.0, min(self.leg_raise_height, self.leg_rear_left_z))

        if self.leg_rear_right_raised:
            self.leg_rear_right_x += leg_raised_vel_x * self.movement_speed * time_elapsed
            self.leg_rear_right_y += leg_raised_vel_y * self.movement_speed * time_elapsed
            self.leg_rear_right_z += self.leg_raise_speed * time_elapsed
        else:
            self.leg_rear_right_z -= self.leg_raise_speed * time_elapsed
        self.leg_rear_right_z = max(0.0, min(self.leg_raise_height, self.leg_rear_right_z))

        msg = QuadrupedLegPositions()

        x = self.leg_front_right_x
        y = self.leg_front_right_y
        msg.leg_front_right.x = x * math.cos(self.turn_theta) - y * math.sin(self.turn_theta)
        msg.leg_front_right.y = x * math.sin(self.turn_theta) + y * math.cos(self.turn_theta)
        msg.leg_front_right.z = -self.z + self.leg_front_right_z
        tilt_x = msg.leg_front_right.y
        tilt_y = msg.leg_front_right.z
        msg.leg_front_right.y = tilt_x * math.cos(-self.tilt_theta) - tilt_y * math.sin(-self.tilt_theta)
        msg.leg_front_right.z = tilt_x * math.sin(-self.tilt_theta) + tilt_y * math.cos(-self.tilt_theta)

        x = self.leg_front_left_x
        y = self.leg_front_left_y
        msg.leg_front_left.x = x * math.cos(self.turn_theta) - y * math.sin(self.turn_theta)
        msg.leg_front_left.y = x * math.sin(self.turn_theta) + y * math.cos(self.turn_theta)
        msg.leg_front_left.z = -self.z + self.leg_front_left_z
        tilt_x = msg.leg_front_left.y
        tilt_y = msg.leg_front_left.z
        msg.leg_front_left.y = tilt_x * math.cos(-self.tilt_theta) - tilt_y * math.sin(-self.tilt_theta)
        msg.leg_front_left.z = tilt_x * math.sin(-self.tilt_theta) + tilt_y * math.cos(-self.tilt_theta)

        x = self.leg_rear_left_x
        y = self.leg_rear_left_y
        msg.leg_rear_left.x = x * math.cos(self.turn_theta) - y * math.sin(self.turn_theta)
        msg.leg_rear_left.y = x * math.sin(self.turn_theta) + y * math.cos(self.turn_theta)
        msg.leg_rear_left.z = -self.z + self.leg_rear_left_z
        tilt_x = msg.leg_rear_left.y
        tilt_y = msg.leg_rear_left.z
        msg.leg_rear_left.y = tilt_x * math.cos(-self.tilt_theta) - tilt_y * math.sin(-self.tilt_theta)
        msg.leg_rear_left.z = tilt_x * math.sin(-self.tilt_theta) + tilt_y * math.cos(-self.tilt_theta)

        x = self.leg_rear_right_x
        y = self.leg_rear_right_y
        msg.leg_rear_right.x = x * math.cos(self.turn_theta) - y * math.sin(self.turn_theta)
        msg.leg_rear_right.y = x * math.sin(self.turn_theta) + y * math.cos(self.turn_theta)
        msg.leg_rear_right.z = -self.z + self.leg_rear_right_z
        tilt_x = msg.leg_rear_right.y
        tilt_y = msg.leg_rear_right.z
        msg.leg_rear_right.y = tilt_x * math.cos(-self.tilt_theta) - tilt_y * math.sin(-self.tilt_theta)
        msg.leg_rear_right.z = tilt_x * math.sin(-self.tilt_theta) + tilt_y * math.cos(-self.tilt_theta)

        self.publisher_.publish(msg)

        self.last_update_time = now


def main(args=None):
    rclpy.init(args=args)

    quad_mc_motion_test = QuadMCMotionTest()

    rclpy.spin(quad_mc_motion_test)

    quad_mc_motion_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
