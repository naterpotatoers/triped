/*
  leg.hpp
*/

/*
  *Description of Leg class here.*
*/


#include "../include/leg.hpp"


Leg::Leg(int servo_1_pin, int servo_2_pin, int servo_3_pin) {
  this->joint_1_servo_.attach(servo_1_pin);
  this->joint_2_servo_.attach(servo_2_pin);
  this->joint_3_servo_.attach(servo_3_pin);

  this->j1_angle_ = 0.0f;
  this->j2_angle_ = 0.0f;
  this->j3_angle_ = 0.0f;
}


void Leg::WriteAngles(int j1_angle, int j2_angle, int j3_angle) {
  this->joint_1_servo_.write(j1_angle);
  this->joint_2_servo_.write(j2_angle);
  this->joint_3_servo_.write(j3_angle);

  // TODO: remove these assignments when servos wired for feedback signal
  this->j1_angle_ = (float)j1_angle;
  this->j2_angle_ = (float)j2_angle;
  this->j3_angle_ = (float)j3_angle;
}


void Leg::ReadAngles(float &j1_angle, float &j2_angle, float &j3_angle) {
  // TODO: wire servos for feedback signal and read positions
  j1_angle = this->j1_angle_;
  j2_angle = this->j2_angle_;
  j3_angle = this->j3_angle_;
}
