#include <Romi32U4.h>
#include <Chassis.h>
#include <BlueMotor.h>
#include <Rangefinder.h>

class PanelPlacer 
{
    Chassis chassis;
    BlueMotor fourbar;
    Rangefinder ultrasonic;
    Romi32U4ButtonA buttonA;

    enum states { IDLE, RUNNING } state;
    unsigned long clock;

public:
    void init();
    void run();
    void status();

};