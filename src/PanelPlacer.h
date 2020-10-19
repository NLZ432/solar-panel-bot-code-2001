
#include <Romi32U4.h>
#include "RemoteConstants.h"
#include <IRdecoder.h>
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
    PIDController pidRange {0.10, 0, 0};
    
    LineFollower linefollower{0.20, 0, 0, 800};


    enum GoalStates {
        TO_ROOF,
        REMOVE,
        DEPOSIT,
        REPLACE,
        CROSS,
        TEST1,
        TEST2,
        DONE,
        ULTRASONICTEST,
        CALIBRATE_TURN
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
        FIN,
        CALTURN
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
    {   {OPEN_GRIP},
        { WAIT },
        {CLOSE_GRIP},
        { WAIT },
        {DEPO_POSITION, 700},
        {TO_INTERSECTION},
        {DRIVE_DISTANCE, 3},
        {TURN, -95},
        { WAIT },
        {NEXT     }
        }
    };

    goal REMOVE_INST = {
        { {POSITION},
          {OPEN_GRIP},
          {DRIVE_DISTANCE, 2},
          {WAIT},
          {CLOSE_GRIP},
          {WAIT},
          {POSITION, 200},
          {DRIVE_DISTANCE, -5},
          {DEPO_POSITION, 400},
          {NEXT     } } 
    };

    goal DEPOSIT_INST = {
        { 
          {CLOSE_GRIP},
          {DEPO_POSITION, 400},
          {TURN, 210},
          {TO_INTERSECTION},
          {DRIVE_DISTANCE, 2},
          {TURN, -90},
          {WAIT},
          {TO_STATION},
          {DEPO_POSITION, 0},
          {WAIT},
          {OPEN_GRIP},
          {WAIT},
          {CLOSE_GRIP},
          {DEPO_POSITION, 400},
          {DRIVE_DISTANCE, -3},
          {TURN, 180},
          {NEXT     }   }
    };

    goal REPLACE_INST = {
        { 
          {CLOSE_GRIP},
          {POSITION, 220},
          {TO_PANEL},
          {WAIT},
          {OPEN_GRIP},
          {DRIVE_DISTANCE, -7},
          {DEPO_POSITION, 0},
          {CLOSE_GRIP},
          {TURN, 210},
          {WAIT},
          {NEXT     } } 
        };

    goal CROSS_INST = {
        { {TO_INTERSECTION},
          {TURN, 90},
          {DRIVE_DISTANCE, 7},
          {TURN, 90},
          {DRIVE_DISTANCE, 3},
          {SEEK_LINE},
          {TURN, 90},
          {NEXT     },
        } 
    };

    goal TEST1_INST = {
        {   {OPEN_GRIP},
            { WAIT },
            {CLOSE_GRIP},
            { WAIT },
            { NEXT }   }
        };

    goal TEST2_INST = {
    {     {CLOSE_GRIP},
          {DEPO_POSITION, 400},
          {TURN, 210},
          {TO_INTERSECTION},
          {DRIVE_DISTANCE, 2},
          {TURN, -90},
          {WAIT},
          {TO_STATION},
          {DEPO_POSITION, 0},
          {WAIT},
          {OPEN_GRIP},
          {WAIT},
          {CLOSE_GRIP},
          {DEPO_POSITION, 400},
          {DRIVE_DISTANCE, -3},
          {TURN, 180},
          {NEXT     }   }
    };
    
    goal CALIBRATE_TURN_INST = {
        { {WAIT},
          {CALTURN, 180},
          {NEXT     } } 
        };

    goal DONE_INST = {
        { {FIN} } 
        };

    goal ULTRASONICTEST_INST = {
        {
            {TO_INTERSECTION},
            {WAIT},
            {DRIVE_DISTANCE, 2},
            {TURN, -90},
            {TO_PANEL},
            {NEXT     }
        }
    };

    goal goalList[11] = { 
        TO_ROOF_INST,
        REMOVE_INST,
        DEPOSIT_INST,
        REPLACE_INST,
        CROSS_INST,
        TEST1_INST,
        TEST2_INST,
        DONE_INST,
        ULTRASONICTEST_INST,
        CALIBRATE_TURN_INST
        };

    int instNum; //index of current instruction
    enum sides { SIDE_45, SIDE_25 }side;
    bool withCollector = true;
    bool idling;
    unsigned long clock;
    uint16_t servo_pos = 1800;
    bool ESTOPPED = false;

    void nextBehavior();
    void changeGoal();

    int16_t COUNTS_90 = 650;
    long POSITION_THRESHOLD = 30;
    float RIGHT_WEIGHT = 1.0;
    float LEFT_WEIGHT = 1.0;
    float LINEFOLLOWERWEIGHT = 0.35;
    float BASE_EFFORT = 70;
    int STATION_DISTANCE = 16.0f;
    int PANEL_DISTANCE = 16.0f;
    float POSITION_45 = 1900.0f;
    float POSITION_25 = 3200.0f;
    float BATTERY_CONTROL_FACTOR = 1.0;
    float ARMWEIGHT = 1.0f;

public:
    void init();
    void run();
    void status();

};