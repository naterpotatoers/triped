/**
 * inverse_kinematics.hpp
*/


#ifndef TRIPED_INVERSE_KINEMATICS_H_
#define TRIPED_INVERSE_KINEMATICS_H_


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
                       float &x, float &y);


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
    float &j1_angle, float &j2_angle, float &j3_angle);


#endif
