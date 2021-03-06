#include <BlueMotor.h>

char BlueMotor::newValue;
char BlueMotor::oldValue = 0;
const int BlueMotor::encoderPinA;
const int BlueMotor::encoderPinB;
int BlueMotor::errorCount = 0;
long BlueMotor::count = 0;
const char BlueMotor::encoderTable[4][4] = {
        {0, -1, 1, 5},
        {1, 0, 5, -1},
        {-1, 5, 0, 1},
        {5, 1, -1, 0}};

void BlueMotor::mount()
{
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(PWMOutPin, OUTPUT);
    pinMode(encoderPinA, INPUT);
    pinMode(encoderPinB, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderPinA),encoderISR,CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinB),encoderISR,CHANGE);

    pid.setPID(4.0f, 0.5f, 1.0f);

    TCCR1A = 0xA8;
    TCCR1B = 0x11;
    ICR1 = 400;
    OCR1C = 0;
}
void BlueMotor::encoderISR()
{
    newValue = (digitalRead(encoderPinA) << 1) | digitalRead(encoderPinB);
    char val = encoderTable[newValue][oldValue];
    
    if (val == 5) errorCount++;
    else count += val;

    oldValue = newValue;
}
long BlueMotor::getPositionDegrees()
{
    long degrees = (count*DEGREES_PER_COUNT);
    return degrees;
}
long BlueMotor::getPositionCount()
{
    return count;
}
float BlueMotor::getAngularVelocity()
{
    unsigned long current_time = millis();
    if ((current_time - radian_time) > 10)
    {
        omega = (float(count - radian_count)*1000.0f)/float((current_time - radian_time) * COUNTS_PER_RADIAN);
        radian_time = current_time;
        radian_count = count;
    }
    return omega;
}
void BlueMotor::setTargetPosition(float position)
{
    float fcount = position * COUNTS_PER_DEGREE;
    target_count = long(fcount);
}
void BlueMotor::setTargetSpeed(float speed)
{
    target_speed = speed;
}
void BlueMotor::setEffort(int effort)
{
    if (effort > 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else if (effort < 0) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        // digitalWrite(PWMOutPin, HIGH);
    }

    int val = abs(effort) * 4;
    OCR1C = val;
}
int BlueMotor::setEffortWithoutDB(int effort)
{
  int direction = 0;
  if(effort >= 0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    direction = 1;
  }
  else if(effort < 0)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    direction = -1;
  }
  else {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  // digitalWrite(PWMOutPin, HIGH);
  }
  
  uint16_t newEffort = uint16_t(((400.0 - DB) * abs(effort))/400.0 + DB*(effort != 0));
  newEffort = min(newEffort, 400);
  
  //DB*(effort != 0) ensures that if effort is zero, the register is set to zero
  OCR1C = newEffort;
  return int(newEffort)*direction; //multiplied by the direction for sign consistency in graph

}

void BlueMotor::runToTarget()
{
    float effort = pid.calculate(float(count)) * ARMWEIGHT;
    effort = min(max(effort, -400.0f),400.0f);
    setEffortWithoutDB(int(effort));
}
bool BlueMotor::arrived()
{
    return (abs(long(pid.getSetpoint()) - count) < POSITION_THRESHOLD);
}



