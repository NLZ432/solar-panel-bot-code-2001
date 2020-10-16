#include <Arduino.h>
#include <QTRSensors.h>
#include <Romi32U4.h>
#include <PIDController.h>
#include "Chassis.h"


class LineFollower
{

    public:

        LineFollower(float kp, float ki, float kd, int threshold);
        void setSetPoints(int left, int right);
        void lineSetup();

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