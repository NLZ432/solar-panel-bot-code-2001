#include "PanelPlacer.h"

void PanelPlacer::init()
{
    fourbar.mount();
    ultrasonic.wake();

    changeGoal(TG1);

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
        case POSITION:
            if (millis() - clock > 2000)
            {
            fourbar.setEffort(value);
            clock = millis();
            Serial.println(millis());
            next();
            }
            break;
        case IDLE: 
            if (buttonA.isPressed()) { goalState = idleBuffer; }
            break;
    }

}

void PanelPlacer::changeGoal(GoalStates new_goal)
{
    Serial.print("CHANGED TO GOAL ");
    Serial.print(new_goal);
    Serial.print("  BECAUSE INST COUNT = ");
    Serial.println(instNum);
    goalState = new_goal;
    instNum = 0;
}

void PanelPlacer::next()
{
    instNum++;

    //if this was the last instruction, switch the goal state
    if (instNum == goalList[goalState].instructionCount)
    {
        switch(goalState)
        {
            case TG1:
                changeGoal(TG2);
                break;
            case TG2:
                fourbar.setEffort(0);
                delay(500);
                changeGoal(TG1);
                break;
        }
    }
    else 
    {
        Serial.print("CHANGED TO BEHAVIOR ");
        Serial.println(instNum);
    }

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