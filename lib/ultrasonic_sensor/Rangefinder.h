#pragma once
#include <Romi32U4.h>
#include <PIDController.h>

class Rangefinder
{ 
    const int triggerPin = 12;
    static const int echoPin = 0;
    static unsigned long echoTime;
    unsigned long pingTime;
    static void echoISR();
    static float range;
    bool pinging;

    
    public: 
        float getDistanceCM();
        void wake();
        void ping();
        int seek(int setpoint);
        bool arrived();

    private:
        PIDController pid {1.0, 0, 0};
        Romi32U4Motors motors;
};