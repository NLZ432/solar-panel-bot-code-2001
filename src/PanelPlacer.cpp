#include "PanelPlacer.h"


IRDecoder decoder;

void PanelPlacer::init()
{
    goalState = TO_ROOF;
    side = SIDE_45;
    idling = false;

    BASE_EFFORT = BASE_EFFORT * BATTERY_CONTROL_FACTOR;

    chassis.right_90_counts = COUNTS_90;
    chassis.BASE_EFFORT = BASE_EFFORT;
    chassis.RIGHT_WEIGHT = RIGHT_WEIGHT;
    chassis.LEFT_WEIGHT = LEFT_WEIGHT;
    fourbar.ARMWEIGHT = ARMWEIGHT;

    decoder.init();
    fourbar.mount();
    gripper.Init();
    gripper.Attach();
    ultrasonic.wake();
    linefollower.lineSetup();
    linefollower.setParams(BASE_EFFORT, LINEFOLLOWERWEIGHT, 400, 300, RIGHT_WEIGHT, LEFT_WEIGHT);

    Serial.begin(9600);

    delay(3);
    clock = millis();
    Serial.print("initialized");
}

void PanelPlacer::run()
{
    // Serial.print(goalState);
    // if (idling)
    // {
    //     if (buttonC.isPressed()) { idling = false; }
    //     return;
    // }
    if (ESTOPPED)
    {
        if(decoder.getKeyCode() == remoteBack) { ESTOPPED = false; }
        chassis.stop();
        fourbar.setEffort(0);
        return;
    }

    BehaviorStates behavior = goalList[goalState].instructions[instNum].behavior;
    int value = goalList[goalState].instructions[instNum].value;

    switch(behavior)
    {
        case TO_INTERSECTION:{
            
            linefollower.linefollow();

            if(linefollower.intersectionDetected())
            {
                motors.setEfforts(0, 0);
                nextBehavior();
                
            }
            break;
        }

        case DRIVE_DISTANCE:{

            chassis.setTargetDistance(float(value));
            chassis.driveToTarget();
            if (chassis.arrived())
            {
                chassis.stop();
                nextBehavior();  
            }
            break;
        }

        case TO_STATION:{

            pidRange.setSetpoint(STATION_DISTANCE);

            ultrasonic.ping();
            float dist = ultrasonic.getDistanceCM();
            if (dist > pidRange.getSetpoint())
            {
                linefollower.linefollow();
            }
            else
            {
                motors.setEfforts(0,0);
                nextBehavior();
            }
            Serial.println(dist);
            break;

        }
        
        case TO_PANEL:{
            
            pidRange.setSetpoint(PANEL_DISTANCE);

            ultrasonic.ping();
            float dist = ultrasonic.getDistanceCM();
            
            if(dist > pidRange.getSetpoint())
            {
                linefollower.linefollow();
            }
            else
            {
                chassis.stop();
                nextBehavior();
            }
            
            Serial.println(dist);
            break;
        }

        case SEEK_LINE:{

            motors.setEfforts(30,30);
            if(linefollower.lineDetected())
            {
                motors.setEfforts(0, 0);
                nextBehavior();
            }


            break;
        }

        case POSITION:{

            float side_position = (side == SIDE_45) ? POSITION_45 : POSITION_25;
            float val = (side == SIDE_45) ? float(value) : -float(value);
            float position = side_position + val;
            fourbar.pid.setSetpoint(position);

            fourbar.runToTarget();
            if (fourbar.arrived())
            {
                fourbar.setEffort(0);
                nextBehavior();
            }
            break;
        }

        case DEPO_POSITION:{
            fourbar.pid.setSetpoint(float(value));
            fourbar.runToTarget();
            if (fourbar.arrived())
            {
                fourbar.setEffort(0);
                nextBehavior();
            }
            break;
        }

        case CLOSE_GRIP:{
            servo_pos = 1845;
            gripper.Write(servo_pos);
            delay(500);
            nextBehavior();
            break;
        }
        
        case OPEN_GRIP:{

            //closed 1800
            //open 1150
            servo_pos = 1150;
            gripper.Write(servo_pos);
            delay(500);
            nextBehavior();
            break;
        }

        case TURN:{ //positive value = turn left if 45, turn right if 25, vice versa
            
            int angle = value;
            
            if (side == SIDE_25) angle = -angle;

            chassis.setTargetAngle(angle);
            chassis.turnToTarget();

            if (chassis.arrived())
            {
                chassis.stop();
                chassis.resetEncoders();
                nextBehavior();  
            }
            break;
            }

        case WAIT:{
            if (buttonC.isPressed()) 
            { 
                nextBehavior(); 
            }
            if(decoder.getKeyCode() == remotePlayPause)
            {
                nextBehavior();
            }
            ultrasonic.ping();
            float dist = ultrasonic.getDistanceCM();
            Serial.println(dist);
            break;
        }

        case NEXT:
            changeGoal();
            instNum = 0;
            break;

        case FIN:
            if (millis() - clock > 2000)
            {
                clock = millis();
                status();
            }
            break;

        case CALTURN:
            int angle = value;
            
            if (side == SIDE_25) angle = -angle;
            chassis.readEncoders();
            chassis.setTargetAngle(angle);
            chassis.turnToTarget();
            if (chassis.arrived())
            {
                chassis.stop();
                chassis.resetEncoders();
                nextBehavior();  
            }
            break;
    }

    if(decoder.getKeyCode() == remoteStopMode)
    {
        ESTOPPED = true;
        delay(1000);
    }
    
    // if(decoder.getKeyCode() == remoteSetup)
    // {
    //     goalState = RESET;
    // }
}

void PanelPlacer::changeGoal()
{
    switch(goalState)
    {
        case TO_ROOF:
            goalState = withCollector ? REPLACE : REMOVE;
            break;
        case REMOVE:
            goalState = DEPOSIT;
            break;
        case DEPOSIT:
            goalState = TO_ROOF;
            withCollector = true;
            break;
        case REPLACE:
            goalState = CROSS;
            break;
        case CROSS:
            goalState = TO_ROOF;
            side = (side == SIDE_45) ? SIDE_25 : SIDE_45;
            break;

        case TEST1:
            goalState = TEST2;
            side = SIDE_25;
            break;
        case ULTRASONICTEST:
        // case RESET:
        case CALIBRATE_TURN:
        case TEST2:
            goalState = DONE;
            break;
    }
    Serial.print("CHANGED TO GOAL ");
    Serial.println(goalState);
}

void PanelPlacer::nextBehavior()
{
    instNum++;
    chassis.resetEncoders();

    Serial.print("CHANGED TO BEHAVIOR ");
    Serial.println(instNum);

    if (STEP_MODE) { idling = true; }

}

void PanelPlacer::status()
{
    Serial.print("goal: ");
    Serial.print(goalState);
    Serial.print("\tbehavior: ");
    Serial.print(behaviorState);
    Serial.print("\ttime: ");
    Serial.println(clock);
}

