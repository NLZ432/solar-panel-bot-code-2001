#include <Romi32U4.h>
class PIDController
{
    public:
        PIDController();
        PIDController(float p, float i, float d);
        void setPID(float p, float i, float d);
        float calculate(float currentValue);
        void setSetpoint(float target);
        void setTolerance(float tolerance);
        bool onTarget(float currentValue);

        float getSetpoint();
        
        float Kp, Ki, Kd;
    private:
        float previousError;
        float sum;
        float setpoint;
        float tolerance;
        int pidClock = 0;
        float output = 0.0f;

};