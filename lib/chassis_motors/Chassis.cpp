#include "Chassis.h"

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

void Chassis::encoderDriveDistance(float inches)
{
 int DIRECTION = 1; //a coefficient to indicate motor direction
 if (inches != 0) DIRECTION = inches/abs(inches);  
 
 const int DESIRED_COUNT = abs(inches) * COUNTS_PER_INCH;
 const int EFFORT = DIRECTION * 100;
 
 resetEncoders(); 
 motors.setEfforts(EFFORT, EFFORT);
 while(DESIRED_COUNT > abs(encoders.getCountsRight()))
 {
  readEncoders(); //dont stop the motors until the magnitude of the encoder count is the count we desire. 
 }
 motors.setEfforts(0, 0);
}

void Chassis::encoderTurnAngle(float degrees)
{
 int DIRECTION = 1;  //coefficient based on sign of input
 if (degrees != 0) DIRECTION = degrees/abs(degrees);

 const int DESIRED_COUNT = abs(degrees) * COUNTS_PER_DEGREE;
 const int EFFORT = DIRECTION * 100;

 resetEncoders();
 motors.setEfforts(EFFORT, -EFFORT);
 while(DESIRED_COUNT > abs(encoders.getCountsRight()))
 {
   readEncoders(); //dont stop the motors until the magnitude of the encoder count is the count we desire.
 }
 motors.setEfforts(0, 0);
} 