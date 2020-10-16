#include "LineFollower.h"



const uint8_t sensorCount = 4;
const int sensor9 = A6;
const int sensor7 = A2;
const int sensor5 = A3;
const int sensor3 = A4;
int pidtolerance = 10;
int goal = 500;
float propK = 0.12;
float derK = 0.0;
float intK = 0.0;


uint16_t sensorArray[sensorCount];


void LineFollower::lineSetup()
{
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){sensor3, sensor5, sensor7, sensor9}, sensorCount);
    qtr.setEmitterPin(2);

    leftpid.setPID(propK, derK, intK);
    leftpid.setSetpoint(goal);
    leftpid.setTolerance(pidtolerance);

    rightpid.setPID(propK, derK, intK);
    rightpid.setSetpoint(goal);
    rightpid.setTolerance(pidtolerance);
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
    
    int result = (int) leftpid.calculate(sensorArray[0]);
    if(result < 0)
    {
        return 0;
    }
    else if(result > 300)
    {
        return 300;
    }
    return result;
}

int LineFollower::getRightEffort()
{
    int result = (int) rightpid.calculate(sensorArray[3]);
    if(result < 0)
    {
        return 0;
    }
    else if(result > 300)
    {
        return 300;
    }
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




