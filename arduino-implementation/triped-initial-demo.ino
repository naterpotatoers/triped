#include <Arduino.h>
#include "./include/leg.hpp"
#include "./include/inverse_kinematics.hpp"

Leg leg_a;
Leg leg_b;
Leg leg_c;

int loop_delay_time = 2000;

void setup()
{
  Serial.begin(38400);
  Serial.print("Starting robot...");
  leg_a.Initialize(2, 3, 4);
  leg_b.Initialize(5, 6, 7);
  leg_c.Initialize(8, 9, 10);

  pinMode(12, OUTPUT);

  leg_a.WriteAngles(135, 135, 135);
  leg_b.WriteAngles(135, 135, 135);
  leg_c.WriteAngles(135, 135, 135);

  digitalWrite(12, HIGH);
  delay(2000);
}

void loop()
{
  int angle_change;

  // Leg a, joint 3
  angle_change = 15;
  leg_a.WriteAngles(135, 135, 135 + angle_change);
  delay(loop_delay_time);
  leg_a.WriteAngles(135, 135, 135);
  delay(loop_delay_time);
  leg_a.WriteAngles(135, 135, 135 - angle_change);
  delay(loop_delay_time);
  leg_a.WriteAngles(135, 135, 135);
  delay(loop_delay_time);
  // Leg a, joint 2
  angle_change = 15;
  leg_a.WriteAngles(135, 135 + angle_change, 135);
  delay(loop_delay_time);
  leg_a.WriteAngles(135, 135, 135);
  delay(loop_delay_time);
  leg_a.WriteAngles(135, 135 - angle_change, 135);
  delay(loop_delay_time);
  leg_a.WriteAngles(135, 135, 135);
  delay(loop_delay_time);
  // Leg a, joint 1
  angle_change = 15;
  leg_a.WriteAngles(135 + angle_change, 135, 135);
  delay(loop_delay_time);
  leg_a.WriteAngles(135, 135, 135);
  delay(loop_delay_time);
  leg_a.WriteAngles(135 - angle_change, 135, 135);
  delay(loop_delay_time);
  leg_a.WriteAngles(135, 135, 135);
  delay(loop_delay_time);

  // Leg b, joint 3
  angle_change = 15;
  leg_b.WriteAngles(135, 135, 135 + angle_change);
  delay(loop_delay_time);
  leg_b.WriteAngles(135, 135, 135);
  delay(loop_delay_time);
  leg_b.WriteAngles(135, 135, 135 - angle_change);
  delay(loop_delay_time);
  leg_b.WriteAngles(135, 135, 135);
  delay(loop_delay_time);
  // Leg b, joint 2
  angle_change = 15;
  leg_b.WriteAngles(135, 135 + angle_change, 135);
  delay(loop_delay_time);
  leg_b.WriteAngles(135, 135, 135);
  delay(loop_delay_time);
  leg_b.WriteAngles(135, 135 - angle_change, 135);
  delay(loop_delay_time);
  leg_b.WriteAngles(135, 135, 135);
  delay(loop_delay_time);
  // Leg b, joint 1
  angle_change = 15;
  leg_b.WriteAngles(135 + angle_change, 135, 135);
  delay(loop_delay_time);
  leg_b.WriteAngles(135, 135, 135);
  delay(loop_delay_time);
  leg_b.WriteAngles(135 - angle_change, 135, 135);
  delay(loop_delay_time);
  leg_b.WriteAngles(135, 135, 135);
  delay(loop_delay_time);

  // Leg c, joint 3
  angle_change = 15;
  leg_c.WriteAngles(135, 135, 135 + angle_change);
  delay(loop_delay_time);
  leg_c.WriteAngles(135, 135, 135);
  delay(loop_delay_time);
  leg_c.WriteAngles(135, 135, 135 - angle_change);
  delay(loop_delay_time);
  leg_c.WriteAngles(135, 135, 135);
  delay(loop_delay_time);
  // Leg c, joint 2
  angle_change = 15;
  leg_c.WriteAngles(135, 135 + angle_change, 135);
  delay(loop_delay_time);
  leg_c.WriteAngles(135, 135, 135);
  delay(loop_delay_time);
  leg_c.WriteAngles(135, 135 - angle_change, 135);
  delay(loop_delay_time);
  leg_c.WriteAngles(135, 135, 135);
  delay(loop_delay_time);
  // Leg c, joint 1
  angle_change = 15;
  leg_c.WriteAngles(135 + angle_change, 135, 135);
  delay(loop_delay_time);
  leg_c.WriteAngles(135, 135, 135);
  delay(loop_delay_time);
  leg_c.WriteAngles(135 - angle_change, 135, 135);
  delay(loop_delay_time);
  leg_c.WriteAngles(135, 135, 135);
  delay(loop_delay_time);
}
