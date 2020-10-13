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
        REMOVE,
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
        OPEN,
        END
    } behaviorState;

    struct instruction
    {
        BehaviorStates behavior;
        int value; //turn degrees, position degrees, distance, the magnitude of the instruction
    };

    struct goal
    {
        instruction instructions[10];
    };

    goal IDLE_INST = {
        { {IDLE, 0} }
    };

    goal REPLACE = {
        { { POSITION, 10 }, //collector position + 10 (location state)
          { DRIVE, 5 }, //drive forward 5 inches
          { POSITION }, //collector position (location state)
          { OPEN_GRIP } //release collector
          { END          } } 
        };

    goal TESTGOAL2 = {
        { { POSITION,-30 },
          { POSITION,-60 },
          { POSITION,-90 },
          { END          } } 
        };

    goal goalList[3] = { IDLE_INST, TESTGOAL1, TESTGOAL2 };

    int instNum; //index of current instruction
    enum sides { SIDE_45, SIDE_25 }side;
    bool withCollector = false;
    unsigned long clock;
    GoalStates idleBuffer;

    void nextBehavior();
    void changeGoal();

public:
    void init();
    void run();
    void status();

};