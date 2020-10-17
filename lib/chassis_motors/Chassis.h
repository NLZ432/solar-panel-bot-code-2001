#pragma once
#include <Romi32U4.h>

class Chassis
{
public:
 void stop();
 void resetEncoders();
 void readEncoders();
 void setTargetDistance(int counts);
 void setTargetAngle(float degrees);
 void driveToTarget();
 void turnToTarget();
 bool arrived();

 const float wheelDiameter = 2.8;
 const float wheelTrack = 5.75;

 const int CPR = 1440; 
 const int COUNTS_PER_INCH = CPR / (3.14159 * wheelDiameter); //counts per wheel rotation divided by inches per wheel rotation
 const int COUNTS_PER_DEGREE = (COUNTS_PER_INCH * 3.14159 * wheelTrack) / 360; //counts per romi rotation divided by degrees per rotation
 const int16_t right_90_counts = 735; 

 int BASE_EFFORT;
 float weight_right;
 float weight_left;

private:
 Romi32U4Motors motors;
 Romi32U4Encoders encoders;
 int16_t target_count;
}; 