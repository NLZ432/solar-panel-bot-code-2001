#include "PanelPlacer.h"

void PanelPlacer::init()
{
    changeGoal(REMOVE_45);
    
    Serial.begin(9600);

    fourbar.mount();
    ultrasonic.wake();

    delay(3);
    clock = millis();
    Serial.print("initialized");
}

void PanelPlacer::run()
{
    if (behaviorState == IDLE && buttonA.isPressed()) behaviorState = idleBuffer;
    
    switch(goalState)
    {
        case REMOVE_45:
            remove_45_stator();
            break;
        case DEPOSIT_45:
            deposit_45_stator();
            break;
        case PLACE_45:
            place_45_stator();
            break;
        default:
            status();
    }
}

void PanelPlacer::remove_45_stator()
{
    switch(behaviorState)
    {
        case TO_PANEL:
            //if fourbar.position(POSITION_45) <- runs motor, returns true when in position
            //{
                //ultrasonic.ping();
                //float distance = ultrasonic.distance;
                //float error = GRAB_DISTANCE - distance;
                //if linefollower.PID(error) <- runs motor returns true when error < threshold
                //{
                    //stop motors;
                    //changeBehavior(GRAB);
                //} 
            // }
            break;
        
        case GRAB:
        //this could probably be blocking
            //gripper.open();
            //chassis.driveDistance(GRIPPER_APPROACH_DISTANCE);
            //gripper.close();
            //while (!fourbar.position(POSITION_45 + 15)){};
            //chassis.driveDistance(-GRIPPER_APPROACH_DISTANCE);
            //changeGoal(PLACE_45);
            //break;
        default: 
            status();
    }
}

void PanelPlacer::deposit_45_stator()
{
    switch(behaviorState)
    {
        default:
            status();
    }
}

void PanelPlacer::place_45_stator()
{
    switch(behaviorState)
    {
        default:
            status();
    }
}

void PanelPlacer::changeGoal(GoalStates new_goal)
{
    goalState = new_goal;
    switch(goalState)
    {
        case REMOVE_45:
            changeBehavior(TO_PANEL);
            break;
        case DEPOSIT_45:
            behaviorRegister = TO_INTERSECTION; //head to intersection after turn
            memoryRegister = 180; //turn 180 cc
            changeBehavior(TURN);
            break;
        case PLACE_45:
            behaviorRegister = TO_INTERSECTION;
            memoryRegister = 180;
            changeBehavior(TURN);
            break;
    }
}

void PanelPlacer::changeBehavior(BehaviorStates new_behavior)
{
    idleBuffer = new_behavior;
    behaviorState = STEP_MODE ? IDLE : new_behavior;
    chassis.resetEncoders();
}

void PanelPlacer::status()
{
    clock = millis();

    Serial.print("goal: ");
    Serial.print(goalState);
    Serial.print("\tbehavior: ");
    Serial.print(behaviorState);
    Serial.print("\ttime: ");
    Serial.println(clock);
}