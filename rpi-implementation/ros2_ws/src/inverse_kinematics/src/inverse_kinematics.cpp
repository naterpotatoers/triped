#include <chrono>
#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "servo_interfaces/msg/leg_joint_angles.hpp"
#include "servo_interfaces/msg/quadruped_joint_angles.hpp"
#include "servo_interfaces/msg/quadruped_leg_positions.hpp"
#include "inverse_kinematics/inverse_kinematics_lib.hpp"

using std::placeholders::_1;
using namespace servo_interfaces::msg;

class QuadrupedInverseKinematics : public rclcpp::Node
{
  public:
    QuadrupedInverseKinematics()
    : Node("inverse_kinematics")
    {
      subscription_ = this->create_subscription<QuadrupedLegPositions>(
        "leg_positions", 10, std::bind(&QuadrupedInverseKinematics::leg_positions_callback, this, _1));
      publisher_ = this->create_publisher<QuadrupedJointAngles>("joint_angles", 10);

      RCLCPP_INFO(this->get_logger(), "inverse_kinematics running");
    }

  private:
    void leg_positions_callback(const QuadrupedLegPositions& leg_positions_msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Leg positions received:");
      RCLCPP_INFO(this->get_logger(), "\tFront right: < %.2f, %.2f, %.2f >",
                  leg_positions_msg.leg_front_right.x,
                  leg_positions_msg.leg_front_right.y,
                  leg_positions_msg.leg_front_right.z);
      RCLCPP_INFO(this->get_logger(), "\tFront left: < %.2f, %.2f, %.2f >",
                  leg_positions_msg.leg_front_left.x,
                  leg_positions_msg.leg_front_left.y,
                  leg_positions_msg.leg_front_left.z);
      RCLCPP_INFO(this->get_logger(), "\tRear left: < %.2f, %.2f, %.2f >",
                  leg_positions_msg.leg_rear_left.x,
                  leg_positions_msg.leg_rear_left.y,
                  leg_positions_msg.leg_rear_left.z);
      RCLCPP_INFO(this->get_logger(), "\tRear right: < %.2f, %.2f, %.2f >",
                  leg_positions_msg.leg_rear_right.x,
                  leg_positions_msg.leg_rear_right.y,
                  leg_positions_msg.leg_rear_right.z);

      // Perform inverse kinematics
      auto joint_angles_msg = QuadrupedJointAngles();
      int ik_status;
      bool do_publish = true;
      double j1_angle, j2_angle, j3_angle;
      
      RCLCPP_INFO(this->get_logger(), "Joint angles calculated:");

      ik_status = InverseKinematics(
        leg_positions_msg.leg_front_right.x,
        leg_positions_msg.leg_front_right.y,
        leg_positions_msg.leg_front_right.z,
        0.0, 6.0, 6.0,
        -1.0, -1.0,
        j1_angle, j2_angle, j3_angle);
      if (ik_status != 0) {
        do_publish = false;
        RCLCPP_INFO(this->get_logger(), "target position unreachable");
      }
      RCLCPP_INFO(this->get_logger(), "\tFront right: < %.2f, %.2f, %.2f >",
                  j1_angle, j2_angle, j3_angle);
      joint_angles_msg.leg_front_right.joint1_angle = j1_angle;
      joint_angles_msg.leg_front_right.joint2_angle = j2_angle;
      joint_angles_msg.leg_front_right.joint3_angle = j3_angle;
      ik_status = InverseKinematics(
        leg_positions_msg.leg_front_left.x,
        leg_positions_msg.leg_front_left.y,
        leg_positions_msg.leg_front_left.z,
        0.0, 6.0, 6.0,
        1.0, -1.0,
        j1_angle, j2_angle, j3_angle);
      if (ik_status != 0) {
        do_publish = false;
        RCLCPP_INFO(this->get_logger(), "target position unreachable");
      }
      RCLCPP_INFO(this->get_logger(), "\tFront left: < %.2f, %.2f, %.2f >",
                  j1_angle, j2_angle, j3_angle);
      joint_angles_msg.leg_front_left.joint1_angle = j1_angle;
      joint_angles_msg.leg_front_left.joint2_angle = j2_angle;
      joint_angles_msg.leg_front_left.joint3_angle = j3_angle;
      ik_status = InverseKinematics(
        leg_positions_msg.leg_rear_left.x,
        leg_positions_msg.leg_rear_left.y,
        leg_positions_msg.leg_rear_left.z,
        0.0, 6.0, 6.0,
        1.0, 1.0,
        j1_angle, j2_angle, j3_angle);
      if (ik_status != 0) {
        do_publish = false;
        RCLCPP_INFO(this->get_logger(), "target position unreachable");
      }
      RCLCPP_INFO(this->get_logger(), "\tRear left: < %.2f, %.2f, %.2f >",
                  j1_angle, j2_angle, j3_angle);
      joint_angles_msg.leg_rear_left.joint1_angle = j1_angle;
      joint_angles_msg.leg_rear_left.joint2_angle = j2_angle;
      joint_angles_msg.leg_rear_left.joint3_angle = j3_angle;
      ik_status = InverseKinematics(
        leg_positions_msg.leg_rear_right.x,
        leg_positions_msg.leg_rear_right.y,
        leg_positions_msg.leg_rear_right.z,
        0.0, 6.0, 6.0,
        -1.0, 1.0,
        j1_angle, j2_angle, j3_angle);
      if (ik_status != 0) {
        do_publish = false;
        RCLCPP_INFO(this->get_logger(), "target position unreachable");
      }
      RCLCPP_INFO(this->get_logger(), "\tRear right: < %.2f, %.2f, %.2f >",
                  j1_angle, j2_angle, j3_angle);
      joint_angles_msg.leg_rear_right.joint1_angle = j1_angle;
      joint_angles_msg.leg_rear_right.joint2_angle = j2_angle;
      joint_angles_msg.leg_rear_right.joint3_angle = j3_angle;

      if (do_publish) {
        publisher_->publish(joint_angles_msg);
      }
    }
    rclcpp::Subscription<QuadrupedLegPositions>::SharedPtr subscription_;
    rclcpp::Publisher<QuadrupedJointAngles>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuadrupedInverseKinematics>());
  rclcpp::shutdown();

  return 0;
}
