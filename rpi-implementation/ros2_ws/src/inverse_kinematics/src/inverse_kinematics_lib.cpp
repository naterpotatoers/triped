/**
 * inverse_kinematics.cpp
*/


#include "../include/inverse_kinematics.hpp"
#include <cmath>

//#include <stdio.h>

#define TRIPED_PI 3.14159265359f
#define TRIPED_PI_over_2 1.57079632679f
#define TRIPED_PI_times_2 6.28318530718f


/**
 * CircleIntersection
 * 
 * If the circles do not intersect, x and y are unmodified. If there is exactly
 * one point of intersection, x and y specify that intersection regardless of
 * configuration.
 * 
 * Input values:
 *   c1_x: float: x component of center of circle 1
 *   c1_y: float: y component of center of circle 1
 *   r1: float: radius of circle 1
 *   c2_x: float: x component of center of circle 2
 *   c2_y: float: y component of center of circle 2
 *   r2: float: radius of circle 2
 *   configuration: float: +1 chooses < x, y > such that the sequence
 *                         [c1, < x, y >, c2] is a counter-clockwise angle.
 *                         -1 chooses < x, y > such that the sequence is a
 *                         clockwise angle.
 * 
 * Returns: int: number of intersections [0, 1, 2]
*/
int CircleIntersection(float c1_x, float c1_y, float r1,
                       float c2_x, float c2_y, float r2,
                       float configuration,
                       float &x, float &y) {
  float c_x_distance = c1_x - c2_x;
  float c_y_distance = c1_y - c2_y;
  float inv_sq_center_distance = 1.0f / (c_x_distance * c_x_distance +
                                         c_y_distance * c_y_distance);
  float r1_sq_plus_r2_sq = r1 * r1 + r2 * r2;
  float r1_sq_minus_r2_sq = r1 * r1 - r2 * r2;
  float factor_1 = 0.5f * r1_sq_minus_r2_sq * inv_sq_center_distance;
  float factor_2_a = 2.0f * r1_sq_plus_r2_sq * inv_sq_center_distance;
  float factor_2_b = r1_sq_minus_r2_sq * inv_sq_center_distance;
  factor_2_b *= factor_2_b;
  float factor_2_discriminant = factor_2_a - factor_2_b - 1.0f;
  if (factor_2_discriminant < 0.0f) {
    return 0;
  }
  x = 0.5f * (c1_x + c2_x) + factor_1 * (c2_x - c1_x);
  y = 0.5f * (c1_y + c2_y) + factor_1 * (c2_y - c1_y);
  float factor_2;
  if (factor_2_discriminant > 0.0f) {
    factor_2 = configuration * 0.5f * sqrtf(factor_2_discriminant);
    x += factor_2 * (c2_y - c1_y);
    y += factor_2 * (c1_x - c2_x);
    return 2;
  }
  return 1;
}


/**
 * inverseKinematics
 * 
 * Target TCP is specified relative to the intersection of the J1 axis and its
 * normal plane that contains the J2 axis. +x points to the right side of the
 * robot; +y points in the direction the robot is facing; +z points in the
 * direction projecting "upward" from the robot's top/dorsal plane.
 * 
 * Input values:
 *   x: float: x component of target TCP position
 *   y: float: y component of target TCP position
 *   z: float: z component of target TCP position
 *   joint_1_plane_offset: float: parallel distance between J1 axis and
 *                                the plane defined by J2 and J3
 *   linkage_2_length: float: parallel distance between J2 axis and J3 axis
 *   linkage_3_length: float: distance between J3 axis and TCP
 *   j1_config: float: +1: ... -1: ...
 *   j2_config: float: +1: ... -1: ...
 * 
 * Output parameters:
 *   j1_angle: float: angle in degrees of J1
 *   j2_angle: float: angle in degrees of J2
 *   j3_angle: float: angle in degrees of J3
 * 
 * Returns: int: 0 if successful, -1 if target TCP position is unreachable
*/
int InverseKinematics(
    float x, float y, float z,
    float joint_1_plane_offset, float linkage_2_length, float linkage_3_length,
    float j1_config, float j2_config,
    float &j1_angle, float &j2_angle, float &j3_angle) {
  float j1_x, j1_y; // < x, y, z > projected onto J1 normal plane
  float j2_x, j2_y; // < x, y, z > projected onto J2/J3 normal plane
  float j1_temp_leg_length;
  float x_temp, y_temp;
  int num_intersections;
  // Project target point onto J1 normal plane
  j1_x = x;
  j1_y = z;
  // Calculate j1_angle
  j1_temp_leg_length = sqrtf(j1_x * j1_x + j1_y * j1_y -
                             joint_1_plane_offset * joint_1_plane_offset);
//  printf("InverseKinematics: j1_temp_leg_length = %f\n", j1_temp_leg_length);
  num_intersections = CircleIntersection(0.0f, 0.0f, joint_1_plane_offset,
                                         j1_x, j1_y, j1_temp_leg_length,
                                         j1_config, x_temp, y_temp);
//  printf("InverseKinematics: < x_temp, y_temp > = < %f , %f >\n", x_temp, y_temp);
  if (num_intersections == 0) {
    j1_angle = 0.0f;
    j2_angle = 0.0f;
    j3_angle = 0.0f;
    return -1;
  }
  j1_angle = atan2(y_temp, -j1_config * x_temp);
  //printf("InverseKinematics: j1_angle = %f\n", j1_angle);
  // Project target point onto J2/J3 normal plane
  j2_x = y;
  j2_y = -j1_config * (-y_temp * x + x_temp * z) / sqrtf(x_temp * x_temp + y_temp * y_temp); //sin(-j1_angle) * x + cos(-j1_angle) * z;
  //printf("InverseKinematics: < j2_x, j2_y > = < %f , %f >\n", j2_x, j2_y);
  // Calculate j2_angle and j3_angle
  num_intersections = CircleIntersection(0.0f, 0.0f, linkage_2_length,
                                         j2_x, j2_y, linkage_3_length,
                                         j2_config, x_temp, y_temp);
  //printf("InverseKinematics: < x_temp, y_temp > = < %f , %f >\n", x_temp, y_temp);
  if (num_intersections == 0) {
    j1_angle = 0.0f;
    j2_angle = 0.0f;
    j3_angle = 0.0f;
    return -1;
  }
  j2_angle = atan2(y_temp, x_temp);
  j3_angle = atan2(j2_y - y_temp, j2_x - x_temp) - j2_angle;
  j2_angle += TRIPED_PI_over_2;
  if (j3_angle >= TRIPED_PI) j3_angle -= TRIPED_PI_times_2;
  if (j3_angle <= -TRIPED_PI) j3_angle += TRIPED_PI_times_2;

  return 0;
}
