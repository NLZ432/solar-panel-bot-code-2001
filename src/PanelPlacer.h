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

    Romi32U4ButtonC buttonC;
    PIDController pidRange {1.0, 0, 0};
    
    LineFollower linefollower;

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
        DEPO_POSITION,
        TO_STATION,
        CLOSE_GRIP,
        SEEK_LINE,
        OPEN_GRIP,
        POSITION,
        TO_PANEL,
        TURN,
        WAIT,
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
        instruction instructions[20];
    };

    goal TO_ROOF_INST = {
        { {TO_INTERSECTION},
          {TURN, -90},
          {DRIVE_DISTANCE, 10},
          {NEXT     } } 
    };

    goal REMOVE_INST = {
        { {POSITION, 0},
          {OPEN_GRIP},
          {DRIVE_DISTANCE, 3},
          {CLOSE_GRIP},
          {WAIT},
          {POSITION, 300},
          {DRIVE_DISTANCE, -3},
          {DEPO_POSITION, 0},
          {NEXT     } } 
    };

    goal DEPOSIT_INST = {
        { {DEPO_POSITION, 20},
          {DRIVE_DISTANCE, 3},
          {DEPO_POSITION, 0},
          {WAIT},
          {OPEN_GRIP},
          {WAIT},
          {CLOSE_GRIP},
          {DEPO_POSITION, 20},
          {DRIVE_DISTANCE, -3},
          {NEXT     } } 
    };

    goal REPLACE_INST = {
        { {CLOSE_GRIP},
          {POSITION, 300},
          {DRIVE_DISTANCE, 3},
          {POSITION, 0},
          {WAIT},
          {OPEN_GRIP},
          {DRIVE_DISTANCE, -3},
          {DEPO_POSITION, 0},
          {NEXT     } } 
        };

    goal CROSS_INST = {
        { {TO_INTERSECTION},
          {TURN, 90},
          {DRIVE_DISTANCE, 10},
          {TURN, 90},
          {DRIVE_DISTANCE, 3},
          {SEEK_LINE},
          {TURN, 90},
          {NEXT     },
        } 
    };

    goal TEST1_INST = {
        { {CLOSE_GRIP},
          {POSITION, 300},
          {DRIVE_DISTANCE, 3},
          {POSITION, 0},
          {WAIT},
          {OPEN_GRIP},
          {DRIVE_DISTANCE, -3},
          {DEPO_POSITION, 0},
          {NEXT     } } 
        };

    goal TEST2_INST = {
      { { NEXT         } } 
        };
    
    goal DONE_INST = {
        { {FIN} } 
        };

    goal ULTRASONICTEST_INST = {
        {
            { TO_PANEL},
            { NEXT }
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