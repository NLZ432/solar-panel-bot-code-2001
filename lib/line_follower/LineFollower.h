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
        void setSetPoints(int left, int right);
        void lineSetup();
        void setParams(float kp, float ki, float kd, int threshold);

        void linefollow();

        bool lineDetected();
        bool intersectionDetected();
        int getLeftEffort();
        int getRightEffort();
        int readLeftValue();
        int readRightValue();

    private:
        QTRSensors qtr;
        PIDController leftpid;
        PIDController rightpid;
        Romi32U4Motors motors;
        Rangefinder ultrasonic;
        int basespeed = 50;
        int goal;

        

        









};