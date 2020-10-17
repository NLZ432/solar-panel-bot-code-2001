#include "PanelPlacer.h"

void PanelPlacer::init()
{
    goalState = TEST1;
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
            //chassis.setEfforts(leffort, righfort);
            //if (linefollower.intersection()){
                //chassis.stop();
                //next();
            //}
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
            //float leffort, float righfort = linefollower.getEfforts();
            //int distance = ultrasonic.range();
            //float controlFactor = distancePID(distance)
            //chassis.setEfforts(leffort * controlFactor, righfort * controlFactor);
            //if (abs(distance - STATION_DISTANCE) > DISTANCE_THRESHOLD){
                //chassis.stop();
                //next();
            // }
            break;
        }
        
        case TO_PANEL:{

            //distancePID.setTarget(PANEL_DISTACE)
            //float leffort, float righfort = linefollower.getEfforts();
            //int distance = ultrasonic.range();
            //float controlFactor = distancePID.seek(distance)
            //chassis.setEfforts(leffort * controlFactor, righfort * controlFactor);
            //if (abs(distance - PANEL_DISTANCE) < DISTANCE_THRESHOLD){
                //chassis.stop();
                //next();
            // }
            break;
        }

        case SEEK_LINE:{

            //chassis.setEfforts(30,30);
            //if (linefollower.line()){
                //chassis.stop;
                //next();
            //}
            break;
        }

        case POSITION:{

            float side_position = (side == SIDE_45) ? 2028 : 3200;
            fourbar.pid.setSetpoint(side_position + float(value));

            long count = fourbar.getPositionCount();
            float effort = fourbar.pid.calculate(float(count));
            effort = min(max(effort, -400),400);
            fourbar.setEffort(int(effort));
            Serial.println(effort);
            if (abs(side_position - count) < POSITION_THRESHOLD)
            {
                fourbar.setEffort(0);
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
            side = SIDE_25;
            break;
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