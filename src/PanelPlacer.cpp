#include "PanelPlacer.h"



void PanelPlacer::init()
{
    goalState = ULTRASONICTEST;
    side = SIDE_45;

    idling = false;

    fourbar.mount();
    ultrasonic.wake();
    
    Serial.begin(9600);

    delay(3);
    clock = millis();
    Serial.print("initialized");
}

void PanelPlacer::run()
{
    if (idling)
    {
        if (buttonA.isPressed()) { idling = false; }
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
            chassis.setEfforts(leftEffort, rightEffort);

            //if (linefollower.intersection()){
                //chassis.stop();
                //next();
            //}
            if(linefollower.intersectionDetected())
            {
                chassis.stop();
                nextBehavior();
            }
            break;
        }

        case DRIVE_DISTANCE:{
            //chassis.setDistance(value);
            //chassis.run()
            //if (chassis.arrived())
            //{
                //chassis.stop();
                //next();  
            //}
            break;
        }

        case TO_STATION:{

            //distancePID.setTarget(STATION_DISTACE)
            pidRange.setSetpoint(STATION_DISTANCE);

            //float leffort, float righfort = linefollower.getEfforts();
            //int distance = ultrasonic.range();
            //float controlFactor = distancePID(distance)
            //chassis.setEfforts(leffort * controlFactor, righfort * controlFactor);
            //if (abs(distance - STATION_DISTANCE) > DISTANCE_THRESHOLD){
                //chassis.stop();
                //next();
            // }

            rangefinder.ping();
            if(rangefinder.getDistanceCM() > pidRange.getSetpoint())
            {
                int left = linefollower.getLeftEffort();
                int right = linefollower.getRightEffort();
                chassis.setEfforts(left + 50, right + 50);
                // 50 is the base speed dof the motors; this is adjustable
            }
            else
            {
                chassis.setEfforts(0,0);
                nextBehavior();
            }
            break;
        }
        
        case TO_PANEL:{
            
            //distancePID.setTarget(PANEL_DISTACE)
            pidRange.setSetpoint(PANEL_DISTANCE);

            //float leffort, float righfort = linefollower.getEfforts();
            //int distance = ultrasonic.range();
            //float controlFactor = distancePID.seek(distance)
            //chassis.setEfforts(leffort * controlFactor, righfort * controlFactor);
            //if (abs(distance - PANEL_DISTANCE) < DISTANCE_THRESHOLD){
                //chassis.stop();
                //next();
            // }
            rangefinder.ping();
            if(rangefinder.getDistanceCM() > pidRange.getSetpoint())
            {
                int left = linefollower.getLeftEffort();
                int right = linefollower.getRightEffort();
                chassis.setEfforts(left + 50, right + 50);
                // 50 is the base speed dof the motors; this is adjustable
            }
            else
            {
                chassis.setEfforts(0,0);
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
            chassis.setEfforts(30,30);
            if(linefollower.lineDetected())
            {
                chassis.stop();
                nextBehavior();
            }


            break;
        }

        case POSITION:{
            //side_position = (side == SIDE_45) ? 45 : 25;
            //positionPID.setTarget(side_position + value);
            //long count = fourbar.getCount;
            //effort = positionPID.seek(count);
            //fourbar.setEffort(effort);
            //if (abs(value - count) < POSITION_THRESHOLD){
                // fourbar.setEffort(0);
                //next();
            // }
            
            fourbar.setEffort(value);

            if (millis() - clock > 3000)
            {
                clock = millis();
                nextBehavior();
            }

            break;
        }

        case CLOSE_GRIP:{
           //if (servo.close()
            break;
        }
        
        case OPEN_GRIP:{

            //if (servo.open())
            break;
        }

        case TURN:{ //positive value = turn left if 45, turn right if 25, vice versa
            
            int angle = value;
            
            if (side == SIDE_25) angle = -angle;

            //chassis.setTargetRotation(angle);
            //if (chassis.turn()) next();
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