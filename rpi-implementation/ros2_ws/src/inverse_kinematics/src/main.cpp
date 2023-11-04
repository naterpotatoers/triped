#include "../include/leg.hpp"
#include "../include/inverse_kinematics.hpp"

Leg leg_a(2, 3, 4);
Leg leg_a(5, 6, 7);
Leg leg_a(8, 9, 10);

int loop_delay_time = 2000;

void setup() {
  pinMode(12, OUTPUT);

  leg_a.WriteAngles(135, 135, 135);
  leg_b.WriteAngles(135, 135, 135);
  leg_c.WriteAngles(135, 135, 135);

  digitalWrite(12, HIGH);
  delay(2000);
}

void loop() {
  int angle_change;

  // Leg a, joint 3
  angle_change = 15;
  leg_a.write(135, 135, 135);
  delay(loop_delay_time);
  leg_a.write(135, 135, 135 + angle_change);
  delay(loop_delay_time);
  leg_a.write(135, 135, 135);
  delay(loop_delay_time);
  leg_a.write(135, 135, 135 - angle_change);
  delay(loop_delay_time);
  leg_a.write(135, 135, 135);
  delay(loop_delay_time);
  // Leg a, joint 2
  angle_change = 15;
  leg_a.write(135, 135, 135);
  delay(loop_delay_time);
  leg_a.write(135, 135 + angle_change, 135);
  delay(loop_delay_time);
  leg_a.write(135, 135, 135);
  delay(loop_delay_time);
  leg_a.write(135, 135 - angle_change, 135);
  delay(loop_delay_time);
  leg_a.write(135, 135, 135);
  delay(loop_delay_time);
  // Leg a, joint 1
  angle_change = 15;
  leg_a.write(135, 135, 135);
  delay(loop_delay_time);
  leg_a.write(135 + angle_change, 135, 135);
  delay(loop_delay_time);
  leg_a.write(135, 135, 135);
  delay(loop_delay_time);
  leg_a.write(135 - angle_change, 135, 135);
  delay(loop_delay_time);
  leg_a.write(135, 135, 135);
  delay(loop_delay_time);

  // Leg b, joint 3
  angle_change = 15;
  leg_b.write(135, 135, 135);
  delay(loop_delay_time);
  leg_b.write(135, 135, 135 + angle_change);
  delay(loop_delay_time);
  leg_b.write(135, 135, 135);
  delay(loop_delay_time);
  leg_b.write(135, 135, 135 - angle_change);
  delay(loop_delay_time);
  leg_b.write(135, 135, 135);
  delay(loop_delay_time);
  // Leg b, joint 2
  angle_change = 15;
  leg_b.write(135, 135, 135);
  delay(loop_delay_time);
  leg_b.write(135, 135 + angle_change, 135);
  delay(loop_delay_time);
  leg_b.write(135, 135, 135);
  delay(loop_delay_time);
  leg_b.write(135, 135 - angle_change, 135);
  delay(loop_delay_time);
  leg_b.write(135, 135, 135);
  delay(loop_delay_time);
  // Leg b, joint 1
  angle_change = 15;
  leg_b.write(135, 135, 135);
  delay(loop_delay_time);
  leg_b.write(135 + angle_change, 135, 135);
  delay(loop_delay_time);
  leg_b.write(135, 135, 135);
  delay(loop_delay_time);
  leg_b.write(135 - angle_change, 135, 135);
  delay(loop_delay_time);
  leg_b.write(135, 135, 135);
  delay(loop_delay_time);

  // Leg c, joint 3
  angle_change = 15;
  leg_c.write(135, 135, 135);
  delay(loop_delay_time);
  leg_c.write(135, 135, 135 + angle_change);
  delay(loop_delay_time);
  leg_c.write(135, 135, 135);
  delay(loop_delay_time);
  leg_c.write(135, 135, 135 - angle_change);
  delay(loop_delay_time);
  leg_c.write(135, 135, 135);
  delay(loop_delay_time);
  // Leg c, joint 2
  angle_change = 15;
  leg_c.write(135, 135, 135);
  delay(loop_delay_time);
  leg_c.write(135, 135 + angle_change, 135);
  delay(loop_delay_time);
  leg_c.write(135, 135, 135);
  delay(loop_delay_time);
  leg_c.write(135, 135 - angle_change, 135);
  delay(loop_delay_time);
  leg_c.write(135, 135, 135);
  delay(loop_delay_time);
  // Leg c, joint 1
  angle_change = 15;
  leg_c.write(135, 135, 135);
  delay(loop_delay_time);
  leg_c.write(135 + angle_change, 135, 135);
  delay(loop_delay_time);
  leg_c.write(135, 135, 135);
  delay(loop_delay_time);
  leg_c.write(135 - angle_change, 135, 135);
  delay(loop_delay_time);
  leg_c.write(135, 135, 135);
  delay(loop_delay_time);
}
