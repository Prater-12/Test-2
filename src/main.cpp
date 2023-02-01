#include "constants.h"

int finishedSetup;

unsigned long currentTimestamp;

int irRead;

char currentDirection = 'S';
int motorSetReturn = 0;
int takeSharpTurns = 0;
unsigned long sharpTurnStart = 0;

void pinModeAll(int const arr[], int size, int pin_mode);

int setMotors(char direction, int force);
void setMotorState(int motor, int state);
void setMotorSpeed(int motor, int speed);

void IR_ReadCycle();

void setup()
{
  // IR Setup
  pinModeAll(IR::Pins, 2, INPUT);

  // Setup.. setup
  pinMode(Setup::BUTTON_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // BO Setup
  pinModeAll(Motor::Pins[0], 3, OUTPUT);
  pinModeAll(Motor::Pins[1], 3, OUTPUT);

  delay(1000);

  Serial.begin(9600);

  Setup::lastLeft = millis();

  while (!finishedSetup)
  {
    if (!digitalRead(Setup::BUTTON_PIN))
    {
      currentTimestamp = millis();

      if (currentTimestamp - Setup::lastLeft > 5000)
      {
        finishedSetup = 1;
      }
      else if (currentTimestamp - Setup::lastLeft > 100)
      {
        if (Setup::botMode == 4)
        {
          Setup::botMode = 0;
        }
        else
        {
          Setup::botMode++;
        }

        Setup::Blink.delayMS = Setup::DELAYS[Setup::botMode];
      }

      Setup::lastLeft = millis();
    }

    // Blink
    if (Setup::botMode == 4)
    {
      digitalWrite(LED_BUILTIN, LOW);
    }
    else if (Setup::botMode == 0)
    {
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else if (currentTimestamp - Setup::Blink.prevIterTimestamp >= Setup::Blink.delayMS)
    {
      Setup::currentLEDState = 1 - Setup::currentLEDState;
      digitalWrite(LED_BUILTIN, Setup::currentLEDState);
      Setup::Blink.prevIterTimestamp = currentTimestamp;
    }

    Serial.println(Setup::botMode);
  }
}

void loop()
{

  currentTimestamp = millis();

  IR_ReadCycle();
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
    else if (currentTimestamp - Iteration::Line.prevIterTimestamp >= Iteration::Line.delayMS)
    {

      setMotorSpeed(0, Motor::SPEED);
      setMotorSpeed(1, Motor::SPEED);
      setMotorState(0, Motor::FORWARD);
      setMotorState(1, Motor::FORWARD);
      return 2;
    }
  }

  return 0;
}
