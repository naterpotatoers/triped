import rclpy
from rclpy.node import Node

from servo_interfaces.msg import LegJointAngles, QuadrupedJointAngles

from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/
import time

# Constants
nbPCAServo = 12

# Parameters
MIN_IMP = [500] * nbPCAServo
MAX_IMP = [2500] * nbPCAServo
ACTUATION_RANGE = [270, 270, 270, 270, 270, 270, 270, 270, 270, 270, 270, 270]


class PCA9685Driver(Node):

    def __init__(self):
        super().__init__('pca9685_driver')

        # Initialize servo joint angle subscriber
        self.subscription = self.create_subscription(
                QuadrupedJointAngles,
                'joint_angles',
                self.listener_callback,
                10)

        # Initialize PCA9685 I2C
        self.pca = ServoKit(channels=16)
        for i in range(nbPCAServo):
            self.pca.servo[i].set_pulse_width_range(MIN_IMP[i], MAX_IMP[i])
            self.pca.servo[i].actuation_range = ACTUATION_RANGE[i]

        self.get_logger().info('PCA9685 running')
        self._last_angle_log_time = 0

    def listener_callback(self, msg):
        now = time.perf_counter()
        if now - self._last_angle_log_time >= 1:
            self._last_angle_log_time = now
            # Log received angles
            self.get_logger().info('Servo angles received:')
            self.get_logger().info('\tFront right: < %.2f, %.2f, %.2f >' % (
                msg.leg_front_right.joint1_angle,
                msg.leg_front_right.joint2_angle,
                msg.leg_front_right.joint3_angle,
            ))
            self.get_logger().info('\tFront left: < %.2f, %.2f, %.2f >' % (
                msg.leg_front_left.joint1_angle,
                msg.leg_front_left.joint2_angle,
                msg.leg_front_left.joint3_angle,
            ))
            self.get_logger().info('\tRear left: < %.2f, %.2f, %.2f >' % (
                msg.leg_rear_left.joint1_angle,
                msg.leg_rear_left.joint2_angle,
                msg.leg_rear_left.joint3_angle,
            ))
            self.get_logger().info('\tRear right: < %.2f, %.2f, %.2f >' % (
                msg.leg_rear_right.joint1_angle,
                msg.leg_rear_right.joint2_angle,
                msg.leg_rear_right.joint3_angle,
            ))

        # Send angles to PCA9685
        self.pca.servo[0].angle = msg.leg_front_right.joint1_angle
        self.pca.servo[1].angle = msg.leg_front_right.joint2_angle
        self.pca.servo[2].angle = msg.leg_front_right.joint3_angle
        self.pca.servo[3].angle = msg.leg_front_left.joint1_angle
        self.pca.servo[4].angle = msg.leg_front_left.joint2_angle
        self.pca.servo[5].angle = msg.leg_front_left.joint3_angle
        self.pca.servo[6].angle = msg.leg_rear_left.joint1_angle
        self.pca.servo[7].angle = msg.leg_rear_left.joint2_angle
        self.pca.servo[8].angle = msg.leg_rear_left.joint3_angle
        self.pca.servo[9].angle = msg.leg_rear_right.joint1_angle
        self.pca.servo[10].angle = msg.leg_rear_right.joint2_angle
        self.pca.servo[11].angle = msg.leg_rear_right.joint3_angle


def main(args=None):
    rclpy.init(args=args)

    servo_driver = PCA9685Driver()

    rclpy.spin(servo_driver)

    servo_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
