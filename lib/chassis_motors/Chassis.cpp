#include "Chassis.h"

void Chassis::stop()
{
  motors.setEfforts(0,0);
}

//to reset the encoders before each movement
void Chassis::resetEncoders()
{
 encoders.getCountsAndResetLeft();
 encoders.getCountsAndResetRight();
}

//to print encoder values to the serial monitor
void Chassis::readEncoders()
{
  Serial.print("Left: ");
  Serial.print(encoders.getCountsLeft());
  Serial.print("\t");
  Serial.print("Right: ");
  Serial.println(encoders.getCountsRight());
}

void Chassis::setTargetDistance(int counts)
{
  target_count = counts;
}

void Chassis::driveToTarget()
{
  const int effort = (target_count > 0) ? BASE_EFFORT : -BASE_EFFORT;
  const float right_effort = float(effort) * weight_right;
  const float left_effort = float(effort) * weight_left;
  
  motors.setEfforts(int(left_effort), int(right_effort));
}

bool Chassis::arrived()
{
  return (abs(encoders.getCountsLeft()) > abs(target_count));
}

void Chassis::setTargetAngle(float degrees)
{
  int count = (degrees / 90) * right_90_counts;
  target_count = int(count);
}

void Chassis::turnToTarget()
{
  const int EFFORT = (target_count > 0) ? BASE_EFFORT : -BASE_EFFORT;
  motors.setEfforts(-1 * EFFORT, EFFORT);
}
