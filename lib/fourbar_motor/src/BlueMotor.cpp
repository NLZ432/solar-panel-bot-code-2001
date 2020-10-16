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
void BlueMotor::moveTo(long target_position)
{

    int effort = 0;
    long error = (target_position - getPositionDegrees());
    if (error == 0) effort = 0;
    else effort = (error/abs(error)) * 40;
    setEffort(effort);

    while (abs(error) > POSITION_THRESHOLD) {
        error = (target_position - getPositionDegrees());
        Serial.println(error);
    }

    setEffort(0);
}
void BlueMotor::run()
{
    target_speed = positionController();
    int control_effort = speedController(target_speed);

    Serial.println(control_effort);

    setEffort(control_effort);
}
int BlueMotor::speedController(float target_speed){
    static float integral = 0.0f;

    float error = target_speed - getAngularVelocity();
    Serial.print(getAngularVelocity());
    integral = (integral + error) * float(abs(error) < SPEED_I_CUTOFF); //integrate only if error within I range
    int control = int(error * speed_kP) + int(integral * speed_kI);

    control = max( min(control, 100) , -100); //cap below magnitude of 100
    return control;
}
float BlueMotor::positionController()
{
    static long integral = 0;

    long error = target_count - count;
    integral = (integral + error) * (abs(error) < POSITION_I_CUTOFF); //integrate only if error within I range
    float control = (float(error) * position_kP) + (float(integral) * position_kI);
    
    return control;
}


