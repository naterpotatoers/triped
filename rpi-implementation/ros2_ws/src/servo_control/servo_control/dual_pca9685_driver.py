import rclpy
from rclpy.node import Node

from servo_interfaces.msg import QuadrupedJointAngles

from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/


# Constants
nbPCAServo = 12

# Servo parameters
PCA_LEFT_MIN_PULSE_WIDTH = 501
PCA_LEFT_MAX_PULSE_WIDTH = 2496
PCA_RIGHT_MIN_PULSE_WIDTH = 523
PCA_RIGHT_MAX_PULSE_WIDTH = 2605
PCA_SELECT = [
    'right', 'right', 'right',  # front right leg
    'left', 'left', 'left',     # front left leg
    'left', 'left', 'left',     # rear left leg
    'right', 'right', 'right',  # rear right leg
]
PCA_PIN_SELECT = [
    0, 1, 2,  # front right leg
    0, 1, 2,  # front left leg
    3, 4, 5,  # rear left leg
    3, 4, 5,  # rear right leg
]
ACTUATION_RANGE = [
    274.5, 277, 275,    # front right leg
    277, 277.5, 275.5,  # front left leg
    275, 277, 274.25,    # rear left leg
    276, 274.25, 277.5,    # rear right leg
]
SERVO_ZERO = [
    141.5, 86.75, 97,  # front right leg
    137, 186.5, 177.5,  # front left leg
    135.5, 91, 87.75,  # rear left leg
    140.5, 184, 181,    # rear right leg
]


RAD2DEG = 180.0 / 3.141592


class PCA9685Driver(Node):

    def __init__(self):
        super().__init__('dual_pca9685_driver')

        # Initialize servo joint angle subscriber
        self.subscription = self.create_subscription(
                QuadrupedJointAngles,
                'joint_angles',
                self.listener_callback,
                10)

        # Initialize PCA9685 I2C
        self.pca_left = ServoKit(address=64, channels=16, frequency=100)
        self.pca_right = ServoKit(address=65, channels=16, frequency=100)
        # Set chip pulse width ranges, compensating for errors to get approx. 500-2500 microseconds
        for i in range(16):
            self.pca_left.servo[i].set_pulse_width_range(
                PCA_LEFT_MIN_PULSE_WIDTH,
                PCA_LEFT_MAX_PULSE_WIDTH
            )
            self.pca_right.servo[i].set_pulse_width_range(
                PCA_RIGHT_MIN_PULSE_WIDTH,
                PCA_RIGHT_MAX_PULSE_WIDTH,
            )
        # Set servo actuation ranges
        for i in range(nbPCAServo):
            pca_pin = PCA_PIN_SELECT[i]
            if PCA_SELECT[i] == 'left':
                self.pca_left.servo[pca_pin].actuation_range = ACTUATION_RANGE[i]
            else:
                self.pca_right.servo[pca_pin].actuation_range = ACTUATION_RANGE[i]

        self.get_logger().info('dual PCA9685 running')
        self._last_angle_log_time = 0

    def listener_callback(self, msg):
        positions_reachable = True
        positions_reachable = positions_reachable and msg.leg_front_right.position_reachable
        positions_reachable = positions_reachable and msg.leg_front_left.position_reachable
        positions_reachable = positions_reachable and msg.leg_rear_left.position_reachable
        positions_reachable = positions_reachable and msg.leg_rear_right.position_reachable

        if not positions_reachable:
            self.get_logger().info('UNREACHABLE POSITION')

        self.print_str = ''
        angles_valid = True
        if msg.leg_front_right.position_reachable:
            angles_valid = angles_valid and self.get_servo_angle_valid(0, -msg.leg_front_right.joint1_angle)
            angles_valid = angles_valid and self.get_servo_angle_valid(1, msg.leg_front_right.joint2_angle)
            angles_valid = angles_valid and self.get_servo_angle_valid(2, -msg.leg_front_right.joint3_angle)
        if msg.leg_front_left.position_reachable:
            angles_valid = angles_valid and self.get_servo_angle_valid(3, msg.leg_front_left.joint1_angle)
            angles_valid = angles_valid and self.get_servo_angle_valid(4, -msg.leg_front_left.joint2_angle)
            angles_valid = angles_valid and self.get_servo_angle_valid(5, msg.leg_front_left.joint3_angle)
        if msg.leg_rear_left.position_reachable:
            angles_valid = angles_valid and self.get_servo_angle_valid(6, -msg.leg_rear_left.joint1_angle)
            angles_valid = angles_valid and self.get_servo_angle_valid(7, -msg.leg_rear_left.joint2_angle)
            angles_valid = angles_valid and self.get_servo_angle_valid(8, msg.leg_rear_left.joint3_angle)
        if msg.leg_rear_right.position_reachable:
            angles_valid = angles_valid and self.get_servo_angle_valid(9, msg.leg_rear_right.joint1_angle)
            angles_valid = angles_valid and self.get_servo_angle_valid(10, msg.leg_rear_right.joint2_angle)
            angles_valid = angles_valid and self.get_servo_angle_valid(11, -msg.leg_rear_right.joint3_angle)
        #print(self.print_str, flush=True)

        if positions_reachable and angles_valid:
            # Send angles to PCA9685
            self.set_servo_angle(0, -msg.leg_front_right.joint1_angle)
            self.set_servo_angle(1, msg.leg_front_right.joint2_angle)
            self.set_servo_angle(2, -msg.leg_front_right.joint3_angle)
            self.set_servo_angle(3, msg.leg_front_left.joint1_angle)
            self.set_servo_angle(4, -msg.leg_front_left.joint2_angle)
            self.set_servo_angle(5, msg.leg_front_left.joint3_angle)
            self.set_servo_angle(6, -msg.leg_rear_left.joint1_angle)
            self.set_servo_angle(7, -msg.leg_rear_left.joint2_angle)
            self.set_servo_angle(8, msg.leg_rear_left.joint3_angle)
            self.set_servo_angle(9, msg.leg_rear_right.joint1_angle)
            self.set_servo_angle(10, msg.leg_rear_right.joint2_angle)
            self.set_servo_angle(11, -msg.leg_rear_right.joint3_angle)

    def get_servo_angle_valid(self, servo_i, angle_rad):
        self.print_str += '{:<2} {:.2f}'.format(servo_i, SERVO_ZERO[servo_i] + RAD2DEG * angle_rad)
        self.print_str += ' ' * (8 - (len(self.print_str) % 8))
        angle_deg = (SERVO_ZERO[servo_i] + RAD2DEG * angle_rad) % 360.0
        angle_valid = 0.0 <= angle_deg and angle_deg <= ACTUATION_RANGE[servo_i]
        if not angle_valid:
            self.get_logger().info('invalid servo {:<2} angle: {:.2f}'.format(servo_i, angle_deg))
        return angle_valid

    def set_servo_angle(self, servo_i, angle_rad):
        angle_deg = (SERVO_ZERO[servo_i] + RAD2DEG * angle_rad) % 360.0
        angle_deg_clipped = max(0.0, min(ACTUATION_RANGE[servo_i], angle_deg))
        pca_pin = PCA_PIN_SELECT[servo_i]
        if PCA_SELECT[servo_i] == 'left':
            self.pca_left.servo[pca_pin].angle = angle_deg_clipped
        else:
            self.pca_right.servo[pca_pin].angle = angle_deg_clipped
        # self.get_logger().info('servo[%d].angle = %.2f' % (servo_i, angle_deg))


def main(args=None):
    rclpy.init(args=args)

    servo_driver = PCA9685Driver()

    rclpy.spin(servo_driver)

    servo_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
