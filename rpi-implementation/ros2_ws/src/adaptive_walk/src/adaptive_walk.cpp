#include <chrono>
#include <cstdio>
#include <memory>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "servo_interfaces/msg/leg_joint_angles.hpp"
#include "servo_interfaces/msg/quadruped_joint_angles.hpp"
#include "servo_interfaces/msg/quadruped_leg_positions.hpp"
#include "mission_control_interfaces/msg/mission_control_to_robot.hpp"
#include "quadruped_kinematics/quadruped_kinematics_lib.hpp"

using std::placeholders::_1;
using namespace servo_interfaces::msg;
using namespace servo_interfaces::srv;
using namespace mission_control_interfaces::msg;
using namespace geometry_msgs::msg;


#define MIN(a, b) (((a) <= (b)) ? (a) : (b))
#define MAX(a, b) (((a) >= (b)) ? (a) : (b))
#define CLIP_BY_VALUE(x, minval, maxval) MAX((minval), MIN((maxval), (x)))


typedef struct {
  Vector3 leg_front_right;
  Vector3 leg_front_left;
  Vector3 leg_rear_left;
  Vector3 leg_rear_right;
} QuadrupedLegVelocities;


class AdaptiveWalk : public rclcpp::Node
{
  public:
    AdaptiveWalk();

  private:
    void mission_control_callback(const MissionControlToRobot& mc_msg);
    QuadrupedLegPositions get_chassis_leg_pos(const QuadrupedLegPositions& ground_pos) const;
    QuadrupedLegPositions get_ground_leg_pos(const QuadrupedLegPositions& chassis_pos) const;
    Vector3 get_leg_flow(const Vector3& ground_pos, const MissionControlToRobot& mc) const;
    void decay_target_speed(double halt_time);

    void timer_callback();

    rclcpp::Subscription<QuadrupedLegPositions>::SharedPtr subscription_;
    rclcpp::Publisher<QuadrupedLegPositions>::SharedPtr publisher_;
    QuadrupedLegPositions current_positions;  // relative to ground
    QuadrupedLegVelocities current_velocities;  // relative to ground
    QuadrupedJointAngles current_angles;
    MissionControlToRobot target_mc;
    MissionControlToRobot current_mc;

    double last_time;
    double time_elapsed;  // seconds
    double max_lin_speed;
    double max_ang_speed;
    double max_joint_speed;  // maximum speed of joints
    double step_height;  // height from ground to raise foot when stepping
    double step_clearance_height;
    double raised_speed_factor;  // how much faster to move legs that are raised
    int next_raise_leg;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdaptiveWalk>());
  rclcpp::shutdown();

  return 0;
}


AdaptiveWalk::AdaptiveWalk()
: Node("adaptive_walk")
{
  subscription_ = this->create_subscription<MissionControlToRobot>(
    "mission_control_to_robot", 10, std::bind(&AdaptiveWalk::mission_control_callback, this, _1));
  publisher_ = this->create_publisher<QuadrupedLegPositions>("leg_positions", 10);

  max_lin_speed = 0.2;  // inches per second
  max_ang_speed = TRIPED_PI_times_2 * 0.25;  // radians per second
  step_height = 2.0;  // inches
  step_clearance_height = 1.0;  // inches
  next_raise_leg = 0;
  raised_speed_factor = 1.5;

  RCLCPP_INFO(this->get_logger(), "adaptive walker running");
}


void AdaptiveWalk::mission_control_callback(const MissionControlToRobot& mc_msg)
{
  this->target_mc = mc_msg;
}


QuadrupedLegPositions AdaptiveWalk::get_chassis_leg_pos(
  const QuadrupedLegPositions& ground_pos,
  const MissionControlToRobot& mc) const
{
  QuadrupedLegPositions chassis_pos = QuadrupedLegPositions();

  double ground_tilt_y = cos(-mc.tilt);
  double ground_tilt_z = -sin(-mc.tilt);
  double ground_height_y = sin(-mc.tilt);
  double ground_height_z = cos(-mc.tilt);
  double chassis_z;

  chassis_z = ground_pos.leg_front_right.z - mc.height;
  chassis_pos.leg_front_right.x = ground_pos.leg_front_right.x;
  chassis_pos.leg_front_right.y = ground_tilt_y * ground_pos.leg_front_right.y + \
                                  ground_tilt_z * chassis_z;
  chassis_pos.leg_front_right.z = ground_height_y * ground_pos.leg_front_right.y + \
                                  ground_height_z * chassis_z;

  chassis_z = ground_pos.leg_front_left.z - mc.height;
  chassis_pos.leg_front_left.x = ground_pos.leg_front_left.x;
  chassis_pos.leg_front_left.y = ground_tilt_y * ground_pos.leg_front_left.y + \
                                 ground_tilt_z * chassis_z;
  chassis_pos.leg_front_left.z = ground_height_y * ground_pos.leg_front_left.y + \
                                 ground_height_z * chassis_z;

  chassis_z = ground_pos.leg_rear_left.z - mc.height;
  chassis_pos.leg_rear_left.x = ground_pos.leg_rear_left.x;
  chassis_pos.leg_rear_left.y = ground_tilt_y * ground_pos.leg_rear_left.y + \
                                ground_tilt_z * chassis_z;
  chassis_pos.leg_rear_left.z = ground_height_y * ground_pos.leg_rear_left.y + \
                                ground_height_z * chassis_z;

  chassis_z = ground_pos.leg_rear_right.z - mc.height;
  chassis_pos.leg_rear_right.x = ground_pos.leg_rear_right.x;
  chassis_pos.leg_rear_right.y = ground_tilt_y * ground_pos.leg_rear_right.y + \
                                 ground_tilt_z * ground_pos.leg_rear_right.z;
  chassis_pos.leg_rear_right.z = ground_height_y * ground_pos.leg_rear_right.y + \
                                 ground_height_z * ground_pos.leg_rear_right.z;

  return chassis_pos;
}


/**
 * get_ground_leg_pos
 *
 * Transforms the leg positions from being relative to the chassis to being
 * relative to the ground. Transformed positions have x and y components that
 * are independent of chassis height and tilt, and z components that equal
 * the height of each leg off the ground.
 */
QuadrupedLegPositions AdaptiveWalk::get_ground_leg_pos(
  const QuadrupedLegPositions& chassis_pos,
  const MissionControlToRobot& mc) const
{
  QuadrupedLegPositions ground_pos = QuadrupedLegPositions();

  double ground_tilt_y = cos(mc.tilt);
  double ground_tilt_z = -sin(mc.tilt);
  double ground_height_y = sin(mc.tilt);
  double ground_height_z = cos(mc.tilt);

  ground_pos.leg_front_right.x = chassis_pos.leg_front_right.x;
  ground_pos.leg_front_right.y = ground_tilt_y * chassis_pos.leg_front_right.y + \
                                 ground_tilt_z * chassis_pos.leg_front_right.z;
  ground_pos.leg_front_right.z = ground_height_y * chassis_pos.leg_front_right.y + \
                                 ground_height_z * chassis_pos.leg_front_right.z + \
                                 mc.height;

  ground_pos.leg_front_left.x = chassis_pos.leg_front_left.x;
  ground_pos.leg_front_left.y = ground_tilt_y * chassis_pos.leg_front_left.y + \
                                ground_tilt_z * chassis_pos.leg_front_left.z;
  ground_pos.leg_front_left.z = ground_height_y * chassis_pos.leg_front_left.y + \
                                ground_height_z * chassis_pos.leg_front_left.z + \
                                mc.height;

  ground_pos.leg_rear_left.x = chassis_pos.leg_rear_left.x;
  ground_pos.leg_rear_left.y = ground_tilt_y * chassis_pos.leg_rear_left.y + \
                               ground_tilt_z * chassis_pos.leg_rear_left.z;
  ground_pos.leg_rear_left.z = ground_height_y * chassis_pos.leg_rear_left.y + \
                               ground_height_z * chassis_pos.leg_rear_left.z + \
                               mc.height;

  ground_pos.leg_rear_right.x = chassis_pos.leg_rear_right.x;
  ground_pos.leg_rear_right.y = ground_tilt_y * chassis_pos.leg_rear_right.y + \
                                ground_tilt_z * chassis_pos.leg_rear_right.z;
  ground_pos.leg_rear_right.z = ground_height_y * chassis_pos.leg_rear_right.y + \
                                ground_height_z * chassis_pos.leg_rear_right.z + \
                                mc.height;

  return ground_pos;
}


Vector3 AdaptiveWalk::get_leg_flow(const Vector3& ground_pos, const MissionControlToRobot& mc) const
{
  // Flow points in the opposite direction of robot velocity
  // get_leg_flow operates on foot positions that are relative to the chassis
  Vector3 flow;
  flow.x = 0.0;
  flow.y = 0.0;
  flow.z = 0.0;

  // Apply linear velocity
  flow.x -= mc.speed_right;
  flow.y -= mc.speed_forward;

  // Apply angular velocity
  flow.x -= mc.speed_angular * -ground_pos.y;
  flow.y -= mc.speed_angular * ground_pos.x;

  return flow;
}


void AdaptiveWalk::decay_target_speed(double halt_time)
{
  // Decay target speed over time when not being updated by mission control
  // to mitigate connection-loss issues
  double new_x_vel = this->target_mc.speed_right;
  double new_y_vel = this->target_mc.speed_forward;
  double new_ang_vel = this->target_mc.speed_angular;
  new_x_vel -= this->max_lin_speed * time_elapsed / halt_time;
  new_y_vel -= this->max_lin_speed * time_elapsed / halt_time;
  new_ang_vel = this->max_ang_speed * time_elapsed / halt_time;
  double new_dot_old = new_x_vel * this->target_mc.speed_right + \
                       new_y_vel * this->target_mc.speed_forward;
  if (new_dot_old < 0.0) {
    new_x_vel = 0.0;
    new_y_vel = 0.0;
  }
  if (new_ang_vel * this->target_mc.speed_angular < 0.0) {
    new_ang_vel = 0.0;
  }
  this->target_mc.speed_right = new_x_vel;
  this->target_mc.speed_forward = new_y_vel;
  this->target_mc.speed_angular = new_ang_vel;
}


void AdaptiveWalk::timer_callback() {
  this->time_elapsed = 0.0;

  // Update current mission control
  this->current_mc = this->target_mc;

  // Iteratively find flow at each leg, increment position, apply IK, check reachable
  // First leg not reachable is next step target
  QuadrupedLegPositions curr_leg_pos = this->current_positions;
  QuadrupedJointAngles curr_joint_angles;
  QuadrupedLegVelocities leg_flows;
  double flow_step_size = 0.0625;  // inches
  int max_flow_steps = 1000;
  double flow_distance_traveled = 0.0;
  bool leg_front_right_limit_error = false;
  bool leg_front_left_limit_error = false;
  bool leg_rear_left_limit_error = false;
  bool leg_rear_right_limit_error = false;
  double leg_front_right_distance_to_limit_error = 0.0;
  double leg_front_left_distance_to_limit_error = 0.0;
  double leg_rear_left_distance_to_limit_error = 0.0;
  double leg_rear_right_distance_to_limit_error = 0.0;
  int leg_soonest_limit_error = 0;
  for (int i = 0; i < max_flow_steps; i++) {
    curr_joint_angles = apply_inverse_kinematics(get_chassis_leg_pos(curr_leg_pos));

    if (!curr_joint_angles.leg_front_right.position_reachable) {
      leg_front_right_limit_error = true;
      leg_soonest_limit_error = 1;
    }
    if (!curr_joint_angles.leg_front_left.position_reachable) {
      leg_front_left_limit_error = true;
      leg_soonest_limit_error = 2;
    }
    if (!curr_joint_angles.leg_rear_left.position_reachable) {
      leg_rear_left_limit_error = true;
      leg_soonest_limit_error = 3;
    }
    if (!curr_joint_angles.leg_rear_right.position_reachable) {
      leg_rear_right_limit_error = true;
      leg_soonest_limit_error = 4;
    }

    if (!leg_front_right_limit_error) {
      leg_flows.leg_front_right = get_leg_flow(curr_leg_pos.leg_front_right,
                                               this->current_mc);
      curr_leg_pos.leg_front_right.x += leg_flows.leg_front_right.x * flow_step_size;
      curr_leg_pos.leg_front_right.y += leg_flows.leg_front_right.y * flow_step_size;
      curr_leg_pos.leg_front_right.z += leg_flows.leg_front_right.z * flow_step_size;
      leg_front_right_distance_to_limit_error += flow_step_size;
    }
    if (!leg_front_left_limit_error) {
      leg_flows.leg_front_left = get_leg_flow(curr_leg_pos.leg_front_left,
                                              this->current_mc);
      curr_leg_pos.leg_front_left.x += leg_flows.leg_front_left.x * flow_step_size;
      curr_leg_pos.leg_front_left.y += leg_flows.leg_front_left.y * flow_step_size;
      curr_leg_pos.leg_front_left.z += leg_flows.leg_front_left.z * flow_step_size;
      leg_front_left_distance_to_limit_error += flow_step_size;
    }
    if (!leg_rear_left_limit_error) {
      leg_flows.leg_rear_left = get_leg_flow(curr_leg_pos.leg_rear_left,
                                             this->current_mc);
      curr_leg_pos.leg_rear_left.x += leg_flows.leg_rear_left.x * flow_step_size;
      curr_leg_pos.leg_rear_left.y += leg_flows.leg_rear_left.y * flow_step_size;
      curr_leg_pos.leg_rear_left.z += leg_flows.leg_rear_left.z * flow_step_size;
      leg_rear_left_distance_to_limit_error += flow_step_size;
    }
    if (!leg_rear_right_limit_error) {
      leg_flows.leg_rear_right = get_leg_flow(curr_leg_pos.leg_rear_right,
                                              this->current_mc);
      curr_leg_pos.leg_rear_right.x += leg_flows.leg_rear_right.x * flow_step_size;
      curr_leg_pos.leg_rear_right.y += leg_flows.leg_rear_right.y * flow_step_size;
      curr_leg_pos.leg_rear_right.z += leg_flows.leg_rear_right.z * flow_step_size;
      leg_rear_right_distance_to_limit_error += flow_step_size;
    }

    if (soonest_limit_error != 0) break;
  }

  // Calculate flows for each leg
  leg_flows.leg_front_right = get_leg_flow(curr_leg_pos.leg_front_right,
                                           this->current_mc);
  leg_flows.leg_front_left = get_leg_flow(curr_leg_pos.leg_front_left,
                                           this->current_mc);
  leg_flows.leg_rear_left = get_leg_flow(curr_leg_pos.leg_rear_left,
                                           this->current_mc);
  leg_flows.leg_rear_right = get_leg_flow(curr_leg_pos.leg_rear_right,
                                           this->current_mc);

  // Move planted legs with flow and raised legs against flow
  if (current_positions.leg_front_right.z < step_clearance_height) {
    current_velocities.leg_front_right.x = leg_flows.leg_front_right.x;
    current_velocities.leg_front_right.y = leg_flows.leg_front_right.y;
    current_velocities.leg_front_right.z = leg_flows.leg_front_right.z;
  } else {
    current_velocities.leg_front_right.x = leg_flows.leg_front_right.x * raised_speed_factor;
    current_velocities.leg_front_right.y = leg_flows.leg_front_right.y * raised_speed_factor;
    current_velocities.leg_front_right.z = leg_flows.leg_front_right.z * raised_speed_factor;
  }
  if (current_positions.leg_front_left.z < step_clearance_height) {
    current_velocities.leg_front_left.x = leg_flows.leg_front_left.x;
    current_velocities.leg_front_left.y = leg_flows.leg_front_left.y;
    current_velocities.leg_front_left.z = leg_flows.leg_front_left.z;
  } else {
    current_velocities.leg_front_left.x = leg_flows.leg_front_left.x * raised_speed_factor;
    current_velocities.leg_front_left.y = leg_flows.leg_front_left.y * raised_speed_factor;
    current_velocities.leg_front_left.z = leg_flows.leg_front_left.z * raised_speed_factor;
  }
  if (current_positions.leg_rear_left.z < step_clearance_height) {
    current_velocities.leg_rear_left.x = leg_flows.leg_rear_left.x;
    current_velocities.leg_rear_left.y = leg_flows.leg_rear_left.y;
    current_velocities.leg_rear_left.z = leg_flows.leg_rear_left.z;
  } else {
    current_velocities.leg_rear_left.x = leg_flows.leg_rear_left.x * raised_speed_factor;
    current_velocities.leg_rear_left.y = leg_flows.leg_rear_left.y * raised_speed_factor;
    current_velocities.leg_rear_left.z = leg_flows.leg_rear_left.z * raised_speed_factor;
  }
  if (current_positions.leg_rear_right.z < step_clearance_height) {
    current_velocities.leg_rear_right.x = leg_flows.leg_rear_right.x;
    current_velocities.leg_rear_right.y = leg_flows.leg_rear_right.y;
    current_velocities.leg_rear_right.z = leg_flows.leg_rear_right.z;
  } else {
    current_velocities.leg_rear_right.x = leg_flows.leg_rear_right.x * raised_speed_factor;
    current_velocities.leg_rear_right.y = leg_flows.leg_rear_right.y * raised_speed_factor;
    current_velocities.leg_rear_right.z = leg_flows.leg_rear_right.z * raised_speed_factor;
  }

  // Apply leg velocities
  current_positions.leg_front_right.x += current_velocities.leg_front_right.x * time_elapsed;
  current_positions.leg_front_right.y += current_velocities.leg_front_right.y * time_elapsed;
  current_positions.leg_front_right.z += current_velocities.leg_front_right.z * time_elapsed;
  current_positions.leg_front_left.x += current_velocities.leg_front_left.x * time_elapsed;
  current_positions.leg_front_left.y += current_velocities.leg_front_left.y * time_elapsed;
  current_positions.leg_front_left.z += current_velocities.leg_front_left.z * time_elapsed;
  current_positions.leg_rear_left.x += current_velocities.leg_rear_left.x * time_elapsed;
  current_positions.leg_rear_left.y += current_velocities.leg_rear_left.y * time_elapsed;
  current_positions.leg_rear_left.z += current_velocities.leg_rear_left.z * time_elapsed;
  current_positions.leg_rear_right.x += current_velocities.leg_rear_right.x * time_elapsed;
  current_positions.leg_rear_right.y += current_velocities.leg_rear_right.y * time_elapsed;
  current_positions.leg_rear_right.z += current_velocities.leg_rear_right.z * time_elapsed;

  // Transform leg positions from relative to ground to relative to chassis
  QuadrupedLegPositions chassis_pos = get_chassis_leg_pos(curr_leg_pos);

  // Clip leg movement to match slowest servo
  curr_joint_angles = apply_inverse_kinematics(chassis_pos);

  publisher_->publish(chassis_pos);

  decay_target_speed(0.5);
}
