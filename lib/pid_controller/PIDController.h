#pragma once
#include <Romi32U4.h>
class PIDController
{
    public:
        PIDController();
        PIDController(float p, float i, float d);
        void setPID(float p, float i, float d);
        float calculate(float currentValue);
        float ultraCalculate(float currentValue);
        void setSetpoint(float target);
        void setTolerance(float tolerance);
        bool onTarget(float currentValue);
        float getSetpoint();
        float getTolerance();
        

    private:
        float Kp, Ki, Kd;
        float previousError;
        float sum;
        float setpoint;
        float tolerance;



};