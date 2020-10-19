
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
        // RESET
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
    {   { WAIT },
        {DEPO_POSITION, 400},
        {TO_INTERSECTION},
        {DRIVE_DISTANCE, 2},
        {TURN, -70},
        {TO_PANEL},
        {NEXT     }
        }
    };

    goal REMOVE_INST = {
        { {POSITION},
          {OPEN_GRIP},
          {DRIVE_DISTANCE, 5},
          {WAIT},
          {CLOSE_GRIP},
          {WAIT},
          {POSITION, -100},
          {DRIVE_DISTANCE, -5},
          {DEPO_POSITION, 400},
          {NEXT     } } 
    };

    goal DEPOSIT_INST = {
        { {TURN, -150},
          {TO_INTERSECTION},
          {TURN, 90},
          {TO_STATION},
          {DEPO_POSITION, 20},
          {DRIVE_DISTANCE, 3},
          {DEPO_POSITION, 0},
          {WAIT},
          {OPEN_GRIP},
          {WAIT},
          {CLOSE_GRIP},
          {DEPO_POSITION, 20},
          {DRIVE_DISTANCE, -3},
          {TURN, 180},
          {NEXT     } } 
    };

    goal REPLACE_INST = {
        { {CLOSE_GRIP},
          {WAIT},
          {POSITION, 350},
          {DRIVE_DISTANCE, 7},
          {WAIT},
          {POSITION, 0},
          {OPEN_GRIP},
          {DRIVE_DISTANCE, -7},
          {DEPO_POSITION, 0},
          {CLOSE_GRIP},
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
        {   { WAIT },
        {DEPO_POSITION, 400},
        {TO_INTERSECTION},
        {DRIVE_DISTANCE, 2},
        {TURN, -70},
        {TO_PANEL},
        {NEXT     }
        }
        };

    goal TEST2_INST = {
    {   {DEPO_POSITION, 0},
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

    // goal RESET_INST = {
    //     {
    //         {DEPO_POSITION, 0},
    //         {NEXT}
    //     }
    // };

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
    bool withCollector = false;
    bool idling;
    unsigned long clock;
    uint16_t servo_pos = 1800;
    bool ESTOPPED = false;

    void nextBehavior();
    void changeGoal();

    long POSITION_THRESHOLD = 30;
    float RIGHT_WEIGHT = 1.0;
    float LEFT_WEIGHT = 1.0;
    float LINEFOLLOWERWEIGHT = 0.5;
    float BASE_EFFORT = 50;
    int STATION_DISTANCE = 20.0f;
    int PANEL_DISTANCE = 20.0f;
    float POSITION_45 = 1900.0f;
    float POSITION_25 = 3200.0f;

public:
    void init();
    void run();
    void status();

};