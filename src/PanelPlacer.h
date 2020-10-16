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
        TO_ROOF,
        REMOVE,
        DEPOSIT,
        REPLACE,
        CROSS,
        TEST1,
        TEST2,
    } goalState;

    enum BehaviorStates { 
        TO_INTERSECTION,
        DRIVE_DISTANCE,
        TO_STATION,
        CLOSE_GRIP,
        SEEK_LINE,
        OPEN_GRIP,
        POSITION,
        TO_PANEL,
        TURN,
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

    goal TO_ROOF_INST = {
        { {TO_INTERSECTION},
          {TURN, -90},
          {DRIVE_DISTANCE, 10},
          {END     } } 
    };

    goal REMOVE_INST = {
        { {POSITION},
          {END     } } 
    };

    goal DEPOSIT_INST = {
        { {POSITION},
          {END     } } 
    };

    goal REPLACE_INST = {
        { { POSITION, 10 }, //collector position + 10 (location state)
          { DRIVE_DISTANCE, 5 }, //drive forward 5 inches
          { POSITION }, //collector position (location state)
          { OPEN_GRIP }, //release collector
          { END       } } 
        };

    goal CROSS_INST = {
        { {POSITION},
          {END     } } 
    };

    goal TEST1_INST = {
        { { POSITION,90 },
          { POSITION,60 },
          { POSITION,30 },
          { END          } } 
        };

    goal TEST2_INST = {
        { { POSITION,-30 },
          { POSITION,-60 },
          { POSITION,-90 },
          { END          } } 
        };

    goal goalList[8] = { 
        TO_ROOF_INST,
        REMOVE_INST,
        DEPOSIT_INST,
        REPLACE_INST,
        CROSS_INST,
        TEST1_INST,
        TEST2_INST,
        };

    int instNum; //index of current instruction
    enum sides { SIDE_45, SIDE_25 }side;
    bool withCollector = false;
    bool idling;
    unsigned long clock;

    void nextBehavior();
    void changeGoal();

public:
    void init();
    void run();
    void status();

};