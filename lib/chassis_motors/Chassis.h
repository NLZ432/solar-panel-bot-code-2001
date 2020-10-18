#pragma once
#include <Romi32U4.h>

class Chassis
{
public:
 void stop();
 void encoderDriveDistance(float inches);
 void encoderTurnAngle(float degrees);
 void resetEncoders();
 void readEncoders();
 void setTargetDistance(float inches);
 void setTargetAngle(float degrees);    // new
 void driveToTarget();
 void turnToTarget();   // new
 bool arrived();

 const float wheelDiameter = 2.8;
 const float wheelTrack = 5.75;

 const int CPR = 1440; 
 const int COUNTS_PER_INCH = CPR / (3.14159 * wheelDiameter); //counts per wheel rotation divided by inches per wheel rotation
 const int COUNTS_PER_DEGREE = (COUNTS_PER_INCH * 3.14159 * wheelTrack) / 360; //counts per romi rotation divided by degrees per rotation
 const int16_t right_90_counts = 774; 

 int BASE_EFFORT;
 float RIGHT_WEIGHT = 1.2f;
 float LEFT_WEIGHT = 1.4f;

private:
 Romi32U4Motors motors;
 Romi32U4Encoders encoders;
 int16_t target_count;
}; 