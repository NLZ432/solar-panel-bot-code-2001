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
 int16_t right_90_counts; 

 int BASE_EFFORT;
 float RIGHT_WEIGHT;
 float LEFT_WEIGHT;

private:
 Romi32U4Motors motors;
 Romi32U4Encoders encoders;
 int16_t target_count;
}; 