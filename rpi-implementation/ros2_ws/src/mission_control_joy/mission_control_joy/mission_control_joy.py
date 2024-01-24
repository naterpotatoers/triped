import rclpy
from rclpy.node import Node
from mission_control_interfaces.msg import MissionControlToRobot
from sensor_msgs.msg import Joy
from mission_control_interfaces.msg import MissionControlToRobot


class MissionControlJoy(Node):

    def __init__(self):
        super().__init__('mission_control_joy')
        self.subscription_ = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        self.publisher_ = self.create_publisher(
            MissionControlToRobot,
            '/mission_control_to_robot',
            10)
        
        self.get_logger().info('mission_control_joy running')
    
    def joy_callback(self, joy_msg):
        mc_msg = MissionControlToRobot()

        mc_msg.speed_right = -joy_msg.axes[0]
        mc_msg.speed_forward = joy_msg.axes[1]
        mc_msg.speed_angular = -joy_msg.axes[2]
        mc_msg.tilt = joy_msg.axes[3]
        mc_msg.height = (joy_msg.axes[5] - joy_msg.axes[4]) * 0.5

        self.publisher_.publish(mc_msg)


def main(args=None):
    rclpy.init(args=args)

    mission_control_joy = MissionControlJoy()

    rclpy.spin(mission_control_joy)

    mission_control_joy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
