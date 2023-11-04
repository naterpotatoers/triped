/*
  leg.hpp
*/

/*
  *Description of Leg class here.*
*/


#ifndef LEG_H
#define LEG_H


#include <Servo.h>


class Leg {
private:
  Servo joint_1_servo_;
  Servo joint_2_servo_;
  Servo joint_3_servo_;
  int servo_1_feedback_pin_;
  int servo_2_feedback_pin_;
  int servo_3_feedback_pin_;
  float j1_angle_;
  float j2_angle_;
  float j3_angle_;
public:
  Leg(int servo_1_pin, int servo_2_pin, int servo_3_pin);
  void WriteAngles(int j1_angle, int j2_angle, int j3_angle);
  void ReadAngles(float &j1_angle, float &j2_angle, float &j3_angle); // TODO
};


#endif
