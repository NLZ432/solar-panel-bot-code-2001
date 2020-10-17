#include <Arduino.h>
#include <QTRSensors.h>
#include <Romi32U4.h>
#include <PIDController.h>
#include "Chassis.h"


class LineFollower
{

    public:

        LineFollower(float kp, float ki, float kd, int threshold);
        LineFollower();
        void setSetPoints(int left, int right);
        void lineSetup();
        void setParams(float kp, float ki, float kd, int threshold);

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
        int goal;

        

        









};