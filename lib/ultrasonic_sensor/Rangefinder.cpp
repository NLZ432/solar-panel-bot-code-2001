#include "Rangefinder.h"

float Rangefinder::range;
const int Rangefinder::echoPin;
unsigned long Rangefinder::echoTime;

void Rangefinder::wake()
{
    attachInterrupt(digitalPinToInterrupt(echoPin),echoISR,CHANGE);
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinging = false;
    pingTime = 0;
    range = 432; //set the range so its defined lol
} 
void Rangefinder::echoISR()
{
    if (digitalRead(echoPin))
    {
        //if the pin has gone HIGH, record the time
        echoTime = millis();
    }
    else
    {
        range = (millis() - echoTime) * 17; 
        //if the pin has gone LOW, multiply the time it was HIGH by half the ~speed
        //of sound in cm/ms to get the ~range in cm.
    }
}
void Rangefinder::ping()
{
    if (!pinging && millis() - pingTime > 100) //if 100ms has passed since last ping, begin the ping, and save the time. 
    {
        digitalWrite(triggerPin, HIGH);
        pingTime = millis();
        pinging = true;
    }

    if (pinging && (millis() - pingTime > 0.01)) //if we have been firing the ping for 10us, stop
    {
        digitalWrite(triggerPin, LOW);
        pinging = false;
    }
}
float Rangefinder::getDistanceCM()
{
    return range;
}