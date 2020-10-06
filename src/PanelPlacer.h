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
        REMOVE_AND_RETURN,
        DEPOSIT,
        REPLACE,
        TO_ROOF,
        CROSS,
        IDLING
    } goalState;

    enum BehaviorStates { 
        TO_INTERSECTION,
        DRIVE_DISTANCE,
        TO_STATION,
        SEEK_LINE,
        TO_PANEL,
        POSITION,
        CLOSE,
        IDLE,
        TURN,
        OPEN
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
    };

    goal IDLE_INST = {
        { {IDLE, 0} }, 1
    };

    goal TESTGOAL1 = {
        { {POSITION,90},
          {POSITION,60},
          {POSITION,30} }, 3 };

    goal TESTGOAL2 = {
        { {POSITION,-30},
          {POSITION,-60},
          {POSITION,-90} }, 3 };

    goal goalList[3] = { IDLE_INST, TESTGOAL1, TESTGOAL2 };

    int instNum; //index of current instruction
    enum sides { SIDE_45, SIDE_25 }side;
    bool withCollector = false;
    unsigned long clock;
    GoalStates idleBuffer;

    void next();
    void changeBehavior(BehaviorStates new_behavior);
    void changeGoal(GoalStates new_goal);

public:
    void init();
    void run();
    void status();

};