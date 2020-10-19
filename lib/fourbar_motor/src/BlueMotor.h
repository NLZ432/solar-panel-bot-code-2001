#pragma once
#include <Romi32U4.h>
#include <PIDController.h>

class BlueMotor 
{
    
    const int PWMOutPin = 11;
    const int AIN2 = 4;
    const int AIN1 = 13;
    
    static void encoderISR();
    static const char encoderTable[4][4];
    static const int encoderPinA = 3;
    static const int encoderPinB = 2;
    static int errorCount;
    static char newValue;
    static char oldValue;
    static long count;

    unsigned long radian_time = 0; //the time of the last radian rotation for velocity calc
    long radian_count = 0; //for determining direction
    float omega = 0; //angular velocity in rad/sec

    const long POSITION_I_CUTOFF = 5000;
    const float position_kP = 0.1f;
    const float position_kI = 0.001f;
    
    const float SPEED_I_CUTOFF = 100;
    const float speed_kP = 2.0f;
    const float speed_kI = 0.9f;

    const int COUNTS_PER_ROTATION = 542;
    const float DEGREES_PER_COUNT = 0.664f;
    const float COUNTS_PER_DEGREE = 1.5f;
    const int COUNTS_PER_RADIAN = 86;

    const int CRUISING_SPEED = 3;
    const int DEFAULT_ACCELERATION = 0.5;
    const int POSITION_THRESHOLD = 50;

    long target_count = 0;
    float target_speed = 0;

    float DB = 33.0f; //motor deadband
    

public: 

    float ARMWEIGHT;

    PIDController pid;

    void setEffort(int effort);
    int setEffortWithoutDB(int effort);

    void setTargetPosition(float position);
    void setTargetSpeed(float speed);

    void runToTarget();
    bool arrived();

    float getAngularVelocity();
    long getPositionDegrees();
    long getPositionCount();

    void mount();
};