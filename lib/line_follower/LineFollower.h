#include <Arduino.h>
#include <QTRSensors.h>
#include <Romi32U4.h>
#include <PIDController.h>
#include <Rangefinder.h>
#include "Chassis.h"


class LineFollower
{

    public:

        LineFollower(float kp, float ki, float kd, int threshold);
        LineFollower();
        void lineSetup();
        void setParams(int base, float bothWeight, int leftSetpoint, int rightSetpoint, float rightWeight, float leftWeight);

        void linefollow();

        bool lineDetected();
        bool intersectionDetected();
        int getLeftEffort();
        int getRightEffort();
        int readLeftValue();
        int readRightValue();

        float WEIGHT_BOTH;
        int basespeed;
    private:
        QTRSensors qtr;
        PIDController leftpid;
        PIDController rightpid;
        Romi32U4Motors motors;
        Rangefinder ultrasonic;
        int goal;

        float WEIGHT_RIGHT;
        float WEIGHT_LEFT;

};