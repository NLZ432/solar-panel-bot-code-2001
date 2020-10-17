#include <Romi32U4.h>
#include <Chassis.h>
#include <BlueMotor.h>
#include <Rangefinder.h>
#include <PIDController.h>
#include <LineFollower.h>
#include <servo32u4.h>

class PanelPlacer 
{

    bool STEP_MODE = false; //will wait for button after each behavior change

    Chassis chassis;
    BlueMotor fourbar;
    Rangefinder ultrasonic;
    Servo32U4 gripper;
    Romi32U4Motors motors;

    Romi32U4ButtonA buttonA;
    PIDController pidRange {1.0, 0, 0};
    
    LineFollower linefollower{0.20, 0, 0, 800};

    int STATION_DISTANCE = 5;
    int PANEL_DISTANCE = 5;
    int basespeed = 50;



    enum GoalStates {
        TO_ROOF,
        REMOVE,
        DEPOSIT,
        REPLACE,
        CROSS,
        TEST1,
        TEST2,
        DONE,
        ULTRASONICTEST
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
        NEXT,
        FIN
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
          {TO_PANEL},
          {NEXT     } } 
    };

    goal REMOVE_INST = {
        { {POSITION},
          {NEXT     } } 
    };

    goal DEPOSIT_INST = {
        { {POSITION},
          {NEXT     } } 
    };

    goal REPLACE_INST = {
        { { POSITION, 10 }, //collector position + 10 (location state)
          { DRIVE_DISTANCE, 5 }, //drive forward 5 inches
          { POSITION }, //collector position (location state)
          { OPEN_GRIP }, //release collector
          { NEXT       } } 
        };

    goal CROSS_INST = {
        { {POSITION},
          {NEXT     } } 
    };

    goal TEST1_INST = {
        { { OPEN_GRIP },
          { CLOSE_GRIP },
          { OPEN_GRIP },
          { CLOSE_GRIP },
          { OPEN_GRIP },
          { CLOSE_GRIP },
          { OPEN_GRIP },
          { CLOSE_GRIP },
          { NEXT        } } 
        };

    goal TEST2_INST = {
      { { NEXT         } } 
        };
    
    goal DONE_INST = {
        { {FIN} } 
        };

    goal ULTRASONICTEST_INST = {
        {
            {TO_INTERSECTION},
            {TURN, -90},
            {TO_PANEL},
            {NEXT     }
        }
    };

    goal goalList[9] = { 
        TO_ROOF_INST,
        REMOVE_INST,
        DEPOSIT_INST,
        REPLACE_INST,
        CROSS_INST,
        TEST1_INST,
        TEST2_INST,
        DONE_INST,
        ULTRASONICTEST_INST
        };

    int instNum; //index of current instruction
    enum sides { SIDE_45, SIDE_25 }side;
    bool withCollector = false;
    bool idling;
    unsigned long clock;
    uint16_t servo_pos = 1800;

    void nextBehavior();
    void changeGoal();

    long POSITION_THRESHOLD = 30;

public:
    void init();
    void run();
    void status();

};