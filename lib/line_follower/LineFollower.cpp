#include "LineFollower.h"


const uint8_t sensorCount = 4;
const int sensor9 = A6;
const int sensor7 = A2;
const int sensor5 = A3;
const int sensor3 = A4;
int pidtolerance = 10;
float propK = 0.12;
float derK = 0.0;
float intK = 0.0;


uint16_t sensorArray[sensorCount];

LineFollower::LineFollower(float kp, float ki, float kd, int threshold)
{
    leftpid.setPID(kp, ki, kd);
    rightpid.setPID(kp, ki, kd);
    goal = threshold;
}
LineFollower::LineFollower(){}

void LineFollower::setSetPoints(int left, int right)
{
    leftpid.setSetpoint(left);
    rightpid.setSetpoint(right);
}

void LineFollower::setParams(float kp, float ki, float kd, int threshold)
{
    leftpid.setPID(kp, ki, kd);
    rightpid.setPID(kp, ki, kd);
    goal = threshold;
}

void LineFollower::lineSetup()
{
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){sensor3, sensor5, sensor7, sensor9}, sensorCount);
    qtr.setEmitterPin(22);
}

bool LineFollower::lineDetected()
{
    qtr.read(sensorArray);

    if(( (int)sensorArray[0] >= goal) || ( (int)sensorArray[3] >= goal ))
    {
        return true;
    }
    return false;
}



bool LineFollower::intersectionDetected()
{
    qtr.read(sensorArray);

    if(( (int)sensorArray[0] >= goal ) && ( (int)sensorArray[3] >= goal ))
    {
        return true;
    }
    return false;
}

int LineFollower::getLeftEffort()
{
    // Reads the right sensor to move the left motor
    qtr.read(sensorArray);
    int result = (int) leftpid.calculate(sensorArray[0]);
    return result;
}

int LineFollower::getRightEffort()
{
    // Reads the left motor to move the right motor
    qtr.read(sensorArray);
    int result = (int) rightpid.calculate(sensorArray[3]);
    return result;
}

int LineFollower::readLeftValue()
{
    qtr.read(sensorArray);
    return (int) sensorArray[0];
}

int LineFollower::readRightValue()
{
    qtr.read(sensorArray);
    return (int) sensorArray[3];
}



