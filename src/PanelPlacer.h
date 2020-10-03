#include <Romi32U4.h>
#include <Chassis.h>
#include <BlueMotor.h>
#include <Rangefinder.h>

class PanelPlacer 
{

    bool STEP_MODE = true; //will wait for button after each behavior change

    Chassis chassis;
    BlueMotor fourbar;
    Rangefinder ultrasonic;
    Romi32U4ButtonA buttonA;

    enum GoalStates {
        REMOVE_45,
        DEPOSIT_45,
        PLACE_45,
        REMOVE_25,
        DEPOSIT_25,
        PLACE_25
    } goalState;
    enum BehaviorStates { 
        IDLE,
        POSITION,
        GRAB,
        TURN,
        PLACE,
        TO_PANEL,
        TO_INTERSECTION,
        TO_STATION
    } behaviorState;
    
    unsigned long clock;
    BehaviorStates idleBuffer;
    BehaviorStates behaviorRegister;
    int memoryRegister;

    void remove_45_stator();
    void deposit_45_stator();
    void place_45_stator();

    void changeBehavior(BehaviorStates new_behavior);
    void changeGoal(GoalStates new_goal);

public:
    void init();
    void run();
    void status();

};