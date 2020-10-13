#include "PanelPlacer.h"

void PanelPlacer::init()
{
    goalState = TO_ROOF;
    side = SIDE_45;

    fourbar.mount();
    ultrasonic.wake();
    Serial.begin(9600);

    delay(3);
    clock = millis();
    Serial.print("initialized");
}

void PanelPlacer::run()
{
    
    BehaviorStates behavior = goalList[goalState].instructions[instNum].behavior;
    int value = goalList[goalState].instructions[instNum].value;

    switch(behavior)
    {
        case TO_INTERSECTION:
            //int leffort, int righfort = linefollower.getEfforts();
            //chassis.setEfforts(leffort, righfort);
            //if (linefollower.intersection()){
                //chassis.stop();
                //next();
            //}
            break;

        case DRIVE_DISTANCE:
            //chassis.setDistance(value);
            //chassis.run()
            //if (chassis.arrived())
            //{
                //chassis.stop();
                //next();  
            //}
            break;

        case TO_STATION:
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
        
        case TO_PANEL:
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

        case SEEK_LINE:
            //chassis.setEfforts(30,30);
            //if (linefollower.line()){
                //chassis.stop;
                //next();
            //}
            break;

        case POSITION:
            //positionPID.setTarget(value);
            //long count = fourbar.getCount;
            //effort = positionPID.seek(count);
            //fourbar.setEffort(effort);
            //if (abs(value - count) < POSITION_THRESHOLD){
                // fourbar.setEffort(0);
                //next();
            // }

        case CLOSE:
            //if (servo.close()
        
        case OPEN:
            //if (servo.open())

        case TURN: //positive value = turn left if 45, turn right if 25, vice versa
            int angle = value;
            
            if (side == SIDE_25) angle = -angle;

            //chassis.setTargetRotation(angle);
            //if (chassis.turn()) next();

        case END:
            changeGoal();
            instNum = 0;
            break;

        case IDLE:
            if (buttonA.isPressed()) { goalState = idleBuffer; }
            break;

        default:
            status(); 
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
    }
    Serial.print("CHANGED TO GOAL ");
    Serial.println(goalState);
}

void PanelPlacer::nextBehavior()
{
    instNum++;

    Serial.print("CHANGED TO BEHAVIOR ");
    Serial.println(instNum);

    if (STEP_MODE) {
        idleBuffer = goalState;
        goalState = IDLING; 
    }

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