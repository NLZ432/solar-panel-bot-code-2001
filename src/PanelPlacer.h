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
         IDLING,
         TG1,
         TG2 
    } goalState;

    enum BehaviorStates { 
        IDLE,
        POSITION,
        OPEN,
        CLOSE,
        TURN,
        PLACE,
        DRIVE_DISTANCE,
        TO_PANEL,
        TO_INTERSECTION,
        TO_STATION
    } behaviorState;

    struct instruction
    {
        BehaviorStates behavior;
        int value; //turn degrees, position degrees, distance, the magnitude of the instruction
    };

    struct goal
    {
        instruction instructions[10];
        int instructionCount;    
        GoalStates state;
    };

    goal IDLE_INST = {
        { {IDLE, 0} }, 1, IDLING
    };

    goal TESTGOAL1 = {
        { {POSITION,90},
          {POSITION,60},
          {POSITION,30} }, 3, TG1 };

    goal TESTGOAL2 = {
        { {POSITION,-30},
          {POSITION,-60},
          {POSITION,-90} }, 3, TG2 };

    int instNum; //index of current instruction
    goal goalList[3] = { IDLE_INST, TESTGOAL1, TESTGOAL2 };

    unsigned long clock;
    GoalStates idleBuffer;

    void remove_45_stator();
    void deposit_45_stator();
    void place_45_stator();

    void next();

    void changeBehavior(BehaviorStates new_behavior);
    void changeGoal(GoalStates new_goal);

public:
    void init();
    void run();
    void status();

};