#include "constants.h"
// #include <NewPing.h> NewPing NewPing(TRIG, ECHO, MAX_DISTANCE)

// Sensors
int irRead;

char currentWheelState = 'S';

unsigned long nextIterTime = 1000;
unsigned long currentTime;

void setMotors(char state);
void assignMotorState(int motor, int state);
void setMotorSpeed(int motor, int speed);

void setup()
{
  // IR Setup
  for (int i = 0; i < 2; i++)
  {
    pinMode(IR::Pins[i], INPUT);
  }

  // BO Setup
  for (int i = 0; i < 3; i++)
  {
    pinMode(Motor::Pins[0][i], OUTPUT);
    pinMode(Motor::Pins[1][i], OUTPUT);
  }
}

void loop()
{
  currentTime = millis(); // Current time-stamp

  if (mainEnable)
  {
    if (currentTime > nextIterTime)
    {
      irRead = digitalRead(IR::Pins[0]) << 1 | digitalRead(IR::Pins[1]);

      setMotors(Motor::wheelStates[irRead]);

      if (currentTime - nextIterTime < 500) // If there is no sudden jump in time
      {
        nextIterTime = currentTime + Iteration::Line; // Wait for 20 ms before next loop
      }
      else
      {
        nextIterTime += Iteration::Line * 2; // Wait for (about) 20 ms (assuming it took 20 ms for this to run)
      }
    }
  }
  else
  {
  }
}

void assignMotorState(int motor, int state)
{
  digitalWrite(Motor::Pins[motor][0], Motor::Configs[state][0]);
  digitalWrite(Motor::Pins[motor][1], Motor::Configs[state][1]);
}

void setMotorSpeed(int motor, int speed)
{
  analogWrite(Motor::Pins[motor][2], speed);
}

void setMotors(char wheelState)
{
  if (wheelState != currentWheelState)
  {
    if (wheelState == 'S')
    {
      setMotorSpeed(0, 0);
      setMotorSpeed(1, 0);
      assignMotorState(0, Motor::STOP);
      assignMotorState(1, Motor::STOP);
    }
    else
    {
      if (currentWheelState == 'S')
      {
        assignMotorState(0, Motor::FORWARD);
        assignMotorState(1, Motor::FORWARD);
      }

      if (wheelState == 'F')
      {
        setMotorSpeed(0, Motor::SPEED);
        setMotorSpeed(1, Motor::SPEED);
      }
      else if (wheelState == 'L')
      {
        setMotorSpeed(0, Motor::TURN_SPEED);
        setMotorSpeed(1, Motor::SPEED);
      }
      else
      {
        setMotorSpeed(0, Motor::SPEED);
        setMotorSpeed(1, Motor::TURN_SPEED);
      }
    }

    currentWheelState = wheelState;
  }
}
