#include "PanelPlacer.h"



void PanelPlacer::init()
{
    goalState = TEST1;
    side = SIDE_45;
    idling = false;

    fourbar.mount();
    gripper.Init();
    gripper.Attach();
    ultrasonic.wake();
    linefollower.setParams(1.0f,0.0f,0.0f,10);
    linefollower.lineSetup();
    
    Serial.begin(9600);

    delay(3);
    clock = millis();
    Serial.print("initialized");
}

void PanelPlacer::run()
{
    if (idling)
    {
        if (buttonC.isPressed()) { idling = false; }
        return;
    }

    BehaviorStates behavior = goalList[goalState].instructions[instNum].behavior;
    int value = goalList[goalState].instructions[instNum].value;

    switch(behavior)
    {
        case TO_INTERSECTION:{
            
            //int leffort, int righfort = linefollower.getEfforts();
            int leftEffort = linefollower.getLeftEffort();
            int rightEffort = linefollower.getRightEffort();

            //chassis.setEfforts(leffort, righfort);
            motors.setEfforts(leftEffort, rightEffort);

            //if (linefollower.intersection()){
                //chassis.stop();
                //next();
            //}
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
                chassis.resetEncoders();
                nextBehavior();  
            }
            break;
        }

        case TO_STATION:{

            // //distancePID.setTarget(STATION_DISTACE)
            // pidRange.setSetpoint(STATION_DISTANCE);

            // //float leffort, float righfort = linefollower.getEfforts();
            // //int distance = ultrasonic.range();
            // //float controlFactor = distancePID(distance)
            // //chassis.setEfforts(leffort * controlFactor, righfort * controlFactor);
            // //if (abs(distance - STATION_DISTANCE) > DISTANCE_THRESHOLD){
            //     //chassis.stop();
            //     //next();
            // // }

            // rangefinder.ping();
            // if(rangefinder.getDistanceCM() > pidRange.getSetpoint())
            // {
            //     int left = linefollower.getLeftEffort();
            //     int right = linefollower.getRightEffort();
            //     chassis.setEfforts(left + 50, right + 50);
            //     // 50 is the base speed dof the motors; this is adjustable
            // }
            // else
            // {
            //     chassis.setEfforts(0,0);
            //     nextBehavior();
            // }

            ultrasonic.ping();
            if(ultrasonic.getDistanceCM() > pidRange.getSetpoint())
            {
                int left = linefollower.getLeftEffort();
                int right = linefollower.getRightEffort();
                motors.setEfforts(left + basespeed, right + basespeed);
            }
            else
            {
                motors.setEfforts(0,0);
                nextBehavior();
            }
            break;
        }
        
        case TO_PANEL:{
            
            // //distancePID.setTarget(PANEL_DISTACE)
            // pidRange.setSetpoint(PANEL_DISTANCE);

            // //float leffort, float righfort = linefollower.getEfforts();
            // //int distance = ultrasonic.range();
            // //float controlFactor = distancePID.seek(distance)
            // //chassis.setEfforts(leffort * controlFactor, righfort * controlFactor);
            // //if (abs(distance - PANEL_DISTANCE) < DISTANCE_THRESHOLD){
            //     //chassis.stop();
            //     //next();
            // // }
            // rangefinder.ping();
            // if(rangefinder.getDistanceCM() > pidRange.getSetpoint())
            // {
            //     int left = linefollower.getLeftEffort();
            //     int right = linefollower.getRightEffort();
            //     chassis.setEfforts(left + 50, right + 50);
            //     // 50 is the base speed dof the motors; this is adjustable
            // }
            // else
            // {
            //     chassis.setEfforts(0,0);
            //     nextBehavior();
            // }
            ultrasonic.ping();
            if(ultrasonic.getDistanceCM() > pidRange.getSetpoint())
            {
                int left = linefollower.getLeftEffort();
                int right = linefollower.getRightEffort();
                motors.setEfforts(left + basespeed, right + basespeed);
            }
            else
            {
                motors.setEfforts(0,0);
                nextBehavior();
            }
            
            break;
        }

        case SEEK_LINE:{

            //chassis.setEfforts(30,30);
            //if (linefollower.line()){
                //chassis.stop;
                //next();
            //}
            motors.setEfforts(30,30);
            if(linefollower.lineDetected())
            {
                motors.setEfforts(0, 0);
                nextBehavior();
            }


            break;
        }

        case POSITION:{

            float side_position = (side == SIDE_45) ? 2028 : 3200;
            fourbar.pid.setSetpoint(side_position + float(value));

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
            Serial.println(servo_pos);
            delay(3000);
            nextBehavior();
            break;
        }
        
        case OPEN_GRIP:{

            //closed 1800
            //open 1250
            servo_pos = 1250;
            gripper.Write(servo_pos);
            Serial.println(servo_pos);
            delay(3000);
            nextBehavior();
            break;
        }

        case TURN:{ //positive value = turn left if 45, turn right if 25, vice versa
            
            int angle = value;
            
            if (side == SIDE_25) angle = -angle;

            //chassis.setTargetRotation(angle);
            chassis.setTargetAngle(angle);

            //if (chassis.turn()) next();
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

            if (buttonC.isPressed()) { nextBehavior(); }
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

    }

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
            break;
        case REPLACE:
            goalState = CROSS;
            break;
        case CROSS:
            goalState = TO_ROOF;
            break;

        case TEST1:
            goalState = TEST2;
            side = SIDE_25;
            break;
        case TEST2:
            goalState = DONE;
            break;
        case ULTRASONICTEST:
            goalState = DONE;
            break;
    }
    Serial.print("CHANGED TO GOAL ");
    Serial.println(goalState);
}

void PanelPlacer::nextBehavior()
{
    instNum++;

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