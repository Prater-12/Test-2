#include "constants.h"
// #include <NewPing.h> NewPing NewPing(TRIG, ECHO, MAX_DISTANCE)

int irRead;

char currentDirection = 'S';
char prevTurn;
int motorNowTurning = 0;

unsigned long nextIterTime = 0;
unsigned long currentTime;

void pinModeAll(int const arr[], int size, int pin_mode);

int setMotors(char state);
void setMotorState(int motor, int state);
void setMotorSpeed(int motor, int speed);

void setup()
{
  // IR Setup
  pinModeAll(IR::Pins, 2, INPUT);

  // BO Setup
  pinModeAll(Motor::Pins[0], 3, OUTPUT);
  pinModeAll(Motor::Pins[1], 3, OUTPUT);
}

void loop()
{

  irRead = (digitalRead(IR::Pins[0]) << 1) | digitalRead(IR::Pins[1]);

  motorNowTurning = setMotors(Motor::MovementDirection[irRead]);

  if (motorNowTurning)
  {
    nextIterTime = millis() + Iteration::Line;
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

int setMotors(char direction)
{
  if (direction != currentDirection)
  {

    if (direction == 'S')
    {
      currentDirection = direction;
      setMotorState(0, Motor::STOP);
      setMotorState(1, Motor::STOP);
    }
    else if (direction == 'R')
    {
      currentDirection = direction;
      setMotorSpeed(0, Motor::FAST_SPEED);
      setMotorSpeed(1, Motor::FAST_SPEED);
      setMotorState(0, Motor::BACKWARD);
      setMotorState(1, Motor::FORWARD);
      return 1;
    }
    else if (direction == 'L')
    {
      currentDirection = direction;
      setMotorSpeed(0, Motor::FAST_SPEED);
      setMotorSpeed(1, Motor::FAST_SPEED);
      setMotorState(0, Motor::FORWARD);
      setMotorState(1, Motor::BACKWARD);
      return 1;
    }
    else if (millis() >= nextIterTime)
    {
      currentDirection = direction;
      setMotorSpeed(0, Motor::SPEED);
      setMotorSpeed(1, Motor::SPEED);
      setMotorState(0, Motor::FORWARD);
      setMotorState(1, Motor::FORWARD);
    }
  }

  return 0;
}
