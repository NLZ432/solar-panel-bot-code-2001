
#include <Arduino.h>
#include <Romi32U4.h>

#include "PIDController.h"

const int PWMOutPin = 11;
const int AIN2 = 4;
const int AIN1 = 13;
const int DB = 33;

long count = 0;
unsigned time = 0;



void PIDController::setPID(float p, float i, float d)
{
    sum = 0;
    previousError = 0;
    Kp = p;
    Ki = i;
    Kd = d;
    setpoint = 0;
}


// Sets the PID constants to 0
PIDController::PIDController() { setPID(0, 0, 0); }

// Sets the PID constants to the desired values
PIDController::PIDController(float  p, float i, float d) { setPID(p, i, d); }

float PIDController::calculate(float currentValue)
{
  float error = setpoint - currentValue;
  float delta = error - previousError;
  previousError = error;
  sum += error;
  return Kp * error + Ki * sum + Kd * delta;
}

void PIDController::setTolerance(float t) { tolerance = t; }

void PIDController::setSetpoint(float target) { setpoint = target; }

bool PIDController::onTarget(float currentValue)
{
  return abs(currentValue - setpoint) <= tolerance;
}

float PIDController::getSetpoint()
{
  return setpoint;
}

float PIDController::getTolerance()
{
  return tolerance;
}

float PIDController::ultraCalculate(float currentValue)
{

  float error = currentValue - setpoint;
  float delta = previousError - error;
  previousError = error;
  sum += error;
  return Kp * error + Ki * sum + Kd * delta;
}

void setEffortwithoutDB(int effort)
{
  if(effort >= 0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else if(effort < 0)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }

  OCR1C = ((400.0 - DB) * abs(effort))/400.0 + DB;

}