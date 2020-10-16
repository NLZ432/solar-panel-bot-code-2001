#include "PIDController.h"

void PIDController::setPID(float p, float i, float d)
{
  sum = 0;
  previousError = 0;
  Kp = p;
  Ki = i;
  Kd = d;
  setpoint = 0;
}
void PIDController::setTolerance(float t) { tolerance = t; }

void PIDController::setSetpoint(float target) { setpoint = target; };

// Sets the PID constants to 0
PIDController::PIDController() { setPID(0, 0, 0); }

// Sets the PID constants to the desired values
PIDController::PIDController(float  p, float i, float d) { setPID(p, i, d); }

float PIDController::calculate(float currentValue)
{
  int currentTime = millis();
  if (currentTime - pidClock > 10)
  {
    float error = setpoint - currentValue;
    float delta = error - previousError;
    previousError = error;
    sum += error;
    output = Kp * error + Ki * sum + Kd * delta;
  }
  return output;
}

bool PIDController::onTarget(float currentValue)
{
  return abs(currentValue - setpoint) <= tolerance;
}

float PIDController::getSetpoint()
{
  return setpoint;
}