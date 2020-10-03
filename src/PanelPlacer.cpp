#include "PanelPlacer.h"

void PanelPlacer::init()
{
    state = IDLE;
    Serial.begin(9600);

    fourbar.mount();
    ultrasonic.wake();

    delay(3);
    clock = millis();
    Serial.print("initialized");
}

void PanelPlacer::run()
{
    switch(state)
    {
        case IDLE:
            if (buttonA.isPressed()) state = RUNNING;
            break;
        case RUNNING:
            status();
            if (!buttonA.isPressed()) state = IDLE;
            break;
        default:
            status();
    }
}

void PanelPlacer::status()
{
    clock = millis();

    Serial.print("state: ");
    Serial.print(state);
    Serial.print("\ttime: ");
    Serial.println(clock);
}