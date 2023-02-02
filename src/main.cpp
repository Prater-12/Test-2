#include "constants.h"
#include <stdlib.h>

unsigned long currentTimestamp;

int irRead;

char currentDirection = 'S';
int motorSetReturn = 0;
int slowerSpeeds = 0;
int IROverride = 0;

SoftwareSerial Bluetooth(0, 1); // RX, TX
int bluetoothData;              // the data given from Computer
char generalDirection = 'S';    // B,S,F

void pinModeAll(int const arr[], int size, int pin_mode);

int setMotors(char direction, int force);
void setMotorState(int motor, int state);
void setMotorSpeed(int motor, int speed);

void IR_ReadCycle();
int moveArm(int min, int max);
int moveArm();
void dealWithFire();

void setup()
{
  pinMode(Arm::PUMP_RELAY, OUTPUT);
  digitalWrite(Arm::PUMP_RELAY, LOW);

  // IR Setup
  pinModeAll(IR::Pins, 2, INPUT);

  // Setup.. setup
  pinMode(Setup::BUTTON_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // BO Setup
  pinModeAll(Motor::Pins[0], 3, OUTPUT);
  pinModeAll(Motor::Pins[1], 3, OUTPUT);

  delay(1000);

  Setup::lastLeft = millis();

  while (Setup::progress != 2) // Big scary setup loop
  {
    currentTimestamp = millis();

    if (!digitalRead(Setup::BUTTON_PIN))
    {

      if (Setup::waitUntilRelease)
      {
        Setup::waitUntilRelease = 0;
      }
      else
      {

        if (currentTimestamp - Setup::lastLeft > Setup::PRESS_TIME)
        {
          if (Setup::progress == 0) // Scroll through possible modes
          {
            if (botMode == Setup::RC)
            {
              botMode = 0;
            }
            else
            {
              botMode++;
            }

            Setup::Blink.delayMS = Setup::DELAYS[botMode];
          }
          else // Scroll through possible angles
          {
            if (Setup::baseAngleIndex == 2)
            {
              Setup::baseAngleIndex = 0;
            }
            else
            {
              Setup::baseAngleIndex++;
            }

            Arm::baseServo.write(Setup::ARM_BASE_ANGLES[Setup::baseAngleIndex]);
          }
        }
      }

      Setup::lastLeft = currentTimestamp;
    }
    else
    {
      if (currentTimestamp - Setup::lastLeft >= Setup::HOLD_TIME && Setup::waitUntilRelease == 0)
      {
        if (Setup::progress == 0)
        {
          if (botMode > 1)
          {
            Arm::baseServo.attach(Arm::BASE_SERVO_PIN);
            Arm::endServo.attach(Arm::END_SERVO_PIN);
            delay(10);
            Arm::baseServo.write(Setup::ARM_BASE_ANGLES[Setup::baseAngleIndex]);
          }
          else if (botMode == Setup::LINE_WITH_OBJ)
          {
            Ultrasonic::servo.attach(Ultrasonic::SERVO_PIN);
          }

          if (botMode & 0b10)
          {
            Setup::progress = 1;
            Setup::Blink.delayMS = 250;
            Setup::waitUntilRelease = 1;
          }
          else
          {
            Setup::progress = 2;
          }
        }
        else
        {
          Setup::progress = 2;
        }
      }
    }

    // Blink
    if (botMode == Setup::RC)
    {
      digitalWrite(LED_BUILTIN, LOW);
    }
    else if (botMode == 0)
    {
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else if (Setup::Blink.checkCoolDown(currentTimestamp))
    {
      Setup::currentLEDState = 1 - Setup::currentLEDState;
      digitalWrite(LED_BUILTIN, Setup::currentLEDState);
      Setup::Blink.prevIterTimestamp = currentTimestamp;
    }
  }

  if (botMode & 0b10)
  {
    pinModeAll(Arm::SensorPins, 2, INPUT);

    if (botMode == Setup::FIRE_WITH_WATER)
    {
      pinMode(Arm::PUMP_RELAY, OUTPUT);
      digitalWrite(Arm::PUMP_RELAY, LOW);
    }
  }
  else if (botMode == Setup::RC)
  {
    Arm::diff = 6;
    Bluetooth.begin(9600);
  }

  digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
  currentTimestamp = millis();

  if (botMode != Setup::RC)
  {
    if ((botMode & 0b10))
    {
      if (Iteration::FlameCheck.checkCoolDown(currentTimestamp))
      {
        slowerSpeeds = 0;
      }

      if (Iteration::Arm.checkCoolDown(currentTimestamp))
      {
        {
          Arm::sensorReadings[0] = analogRead(Arm::SensorPins[0]);
          Arm::sensorReadings[1] = analogRead(Arm::SensorPins[1]);

          if (Arm::sensorReadings[0] + Arm::sensorReadings[1] < 1500)
          {
            Iteration::FlameCheck.prevIterTimestamp = currentDirection;
            slowerSpeeds = 1;
          }

          if (Arm::sensorReadings[0] < 200 || Arm::sensorReadings[1] < 200)
          {
            setMotors('S', 0);
            dealWithFire();
          }

          moveArm();
        }
      }
    }
    IR_ReadCycle();
  }
  else
  {
    if (Bluetooth.available())
    {
      bluetoothData = Bluetooth.read();
      if (bluetoothData == Motor::MovementDirection[0] || bluetoothData == Motor::MovementDirection[1] || bluetoothData == Motor::MovementDirection[2] || bluetoothData == Motor::MovementDirection[3] || bluetoothData == Motor::MovementDirection[4])
      {
        motorSetReturn = setMotors(bluetoothData, 0);

        if (motorSetReturn == 1)
        {
          currentDirection = bluetoothData;
        }
        else if (motorSetReturn == 2)
        {
          currentDirection = bluetoothData;
          generalDirection = bluetoothData;
        }
        Iteration::Line.prevIterTimestamp = currentTimestamp;
      }
      else if (bluetoothData == 'E')
      {
        moveArm();
      }
      else if (bluetoothData == 'C')
      {
        if (Setup::baseAngleIndex == 2)
        {
          Setup::baseAngleIndex = 0;
        }
        else
        {
          Setup::baseAngleIndex++;
        }

        Arm::baseServo.write(Setup::ARM_BASE_ANGLES[Setup::baseAngleIndex]);
      }
      else if (bluetoothData == 'X')
      {
        digitalWrite(Arm::PUMP_RELAY, HIGH);

        currentTimestamp = millis();

        Iteration::FireDouse.prevIterTimestamp = currentTimestamp;

        while (!Iteration::FireDouse.checkCoolDown(currentTimestamp))
        {
          currentTimestamp = millis();
        }

        digitalWrite(Arm::PUMP_RELAY, LOW);
      }
      else if (Iteration::Line.checkCoolDown(currentTimestamp))
      {
        motorSetReturn = setMotors(generalDirection, 0);

        if (motorSetReturn == 2)
        {
          currentDirection = generalDirection;
        }
      }
    }
  }
}

void IR_ReadCycle()
{

  irRead = (digitalRead(IR::Pins[0]) << 1) | digitalRead(IR::Pins[1]); //(PORTC && 0b110) >> 1;

  motorSetReturn = setMotors(Motor::MovementDirection[irRead], 0);

  if (motorSetReturn == 1)
  {
    Iteration::Line.prevIterTimestamp = currentTimestamp;
  }

  if (motorSetReturn)
  {
    currentDirection = Motor::MovementDirection[irRead];
  }
}

int moveArm(int min, int max)
{

  Iteration::Arm.prevIterTimestamp = currentTimestamp;

  Arm::endPosition += Arm::diff;

  if (Arm::endPosition > max || Arm::endPosition < min)
  {
    Arm::diff *= -1;
    Arm::endPosition += Arm::diff;
    if (Arm::endPosition > max)
    {
      Arm::endPosition = max;
      Arm::diff = -1;
    }
    else if (Arm::endPosition < min)
    {
      Arm::endPosition = min;
      Arm::diff = 1;
    }
    Arm::endServo.write(Arm::endPosition);
    return 1;
  }

  Arm::endServo.write(Arm::endPosition);
  return 0;
}

int moveArm()
{
  return moveArm(Arm::MIN_ANGLE, Arm::MAX_ANGLE);
}

void pinModeAll(int const arr[], int size, int pin_mode)
{
  for (int i = 0; i < size; i++)
  {
    pinMode(arr[i], pin_mode);
  }
};

void setMotorState(int motor, int state)
{
  digitalWrite(Motor::Pins[motor][0], Motor::Configs[state][0]);
  digitalWrite(Motor::Pins[motor][1], Motor::Configs[state][1]);
}

void setMotorSpeed(int motor, int speed)
{
  analogWrite(Motor::Pins[motor][2], speed);
}

int setMotors(char direction, int force)
{

  if (direction != currentDirection || force)
  {
    if (direction == 'S')
    {

      setMotorState(0, Motor::STOP);
      setMotorState(1, Motor::STOP);
      return 2;
    }
    else if (direction == 'R')
    {

      setMotorSpeed(0, Motor::FAST_SPEED);
      setMotorSpeed(1, Motor::FAST_SPEED);

      setMotorState(0, Motor::BACKWARD);
      setMotorState(1, Motor::FORWARD);
      return 1;
    }
    else if (direction == 'L')
    {

      setMotorSpeed(0, Motor::FAST_SPEED);
      setMotorSpeed(1, Motor::FAST_SPEED);

      setMotorState(0, Motor::FORWARD);
      setMotorState(1, Motor::BACKWARD);
      return 1;
    }
    else if (direction == 'B')
    {

      setMotorSpeed(0, Motor::FAST_SPEED);
      setMotorSpeed(1, Motor::FAST_SPEED);

      setMotorState(0, Motor::BACKWARD);
      setMotorState(1, Motor::BACKWARD);
      return 2;
    }
    else if (currentTimestamp - Iteration::Line.prevIterTimestamp >= Iteration::Line.delayMS)
    {

      if (slowerSpeeds)
      {
        setMotorSpeed(0, Motor::SLOW_SPEED);
        setMotorSpeed(1, Motor::SLOW_SPEED);
      }
      else
      {
        setMotorSpeed(0, Motor::SPEED);
        setMotorSpeed(1, Motor::SPEED);
      }

      setMotorState(0, Motor::FORWARD);
      setMotorState(1, Motor::FORWARD);
      return 2;
    }
  }

  return 0;
}

void dealWithFire()
{

  int peakPositions[2] = {Arm::endPosition, Arm::endPosition};
  int loopReadings[2] = {1023, 1023};
  int peakReadings[2] = {1023, 1023};
  int sweepCount = 0;
  int notFalseAlarm = 0;
  int equalityPoints = 0;

  Bluetooth.begin(9600);

  while (sweepCount <= 2)
  {
    currentTimestamp = millis();

    if (Iteration::Arm.checkCoolDown(currentTimestamp))
    {
      Iteration::Arm.prevIterTimestamp = currentTimestamp;

      loopReadings[0] = analogRead(Arm::SensorPins[0]);
      loopReadings[1] = analogRead(Arm::SensorPins[1]);

      if (!notFalseAlarm && (loopReadings[0] < 800 || loopReadings[1] < 800))
      {
        notFalseAlarm = 1;
      }

      for (int i = 0; i < 2; i++)
      {
        if (peakReadings[i] - loopReadings[i] > 10)
        {
          equalityPoints = 1;
          peakReadings[i] = loopReadings[i];
          peakPositions[i] = Arm::endPosition;
        }
        else if (abs(loopReadings[i] - peakReadings[i]) <= 10)
        {
          peakPositions[i] = (peakPositions[i] * equalityPoints + Arm::endPosition) / (equalityPoints + 1);
          equalityPoints++;
        }
      }

      sweepCount += moveArm();
    }
  }

  Bluetooth.println(peakPositions[0]);
  Bluetooth.println(peakPositions[1]);

  if (!notFalseAlarm)

    return;

  if (botMode == Setup::FIRE_WITH_WATER)
  {
    digitalWrite(Arm::PUMP_RELAY, HIGH);
  }

  currentTimestamp = millis();

  Iteration::FireDouse.prevIterTimestamp = currentTimestamp;

  while (!Iteration::FireDouse.checkCoolDown(currentTimestamp))
  {
    currentTimestamp = millis();

    if (peakPositions[0] > peakPositions[1])
    {
      moveArm(peakPositions[1], peakPositions[0]);
    }
    else
    {
      moveArm(peakPositions[0], peakPositions[1]);
    }
  }

  if (botMode == Setup::FIRE_WITH_WATER)
  {
    digitalWrite(Arm::PUMP_RELAY, LOW);
  }
}