#include "LineFollower.h"


const uint8_t sensorCount = 2;
const int sensor9 = A2;
const int sensor3 = A3;
int pidtolerance = 10;


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
    qtr.setSensorPins((const uint8_t[]){sensor3, sensor9}, sensorCount);
    // qtr.setEmitterPin(22);
}

bool LineFollower::lineDetected()
{
    qtr.read(sensorArray);

    if(( (int)sensorArray[0] >= goal) || ( (int)sensorArray[1] >= goal ))
    {
        return true;
    }
    return false;
}



bool LineFollower::intersectionDetected()
{
    qtr.read(sensorArray);

    if(( (int)sensorArray[0] >= goal ) && ( (int)sensorArray[1] >= goal ))
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
    int result = (int) rightpid.calculate(sensorArray[1]);
    return result;
}

void LineFollower::linefollow()
{
    int left = getLeftEffort() / 2;
    int right = getRightEffort() / 2;
    motors.setEfforts(left + basespeed, right + basespeed);
}

int LineFollower::readLeftValue()
{
    qtr.read(sensorArray);
    return (int) sensorArray[0];
}

int LineFollower::readRightValue()
{
    qtr.read(sensorArray);
    return (int) sensorArray[1];
}



