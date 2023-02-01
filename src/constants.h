#include <Arduino.h>
#include <Servo.h>
#include <NewPing.h>

#ifndef MS_CONSTANTS
#define MS_CONSTANTS

int const mainEnable = 1;

struct Iterator
{
    unsigned int delayMS;
    unsigned long prevIterTimestamp;

    Iterator(int d)
    {
        delayMS = d;
        prevIterTimestamp = 0;
    };
    Iterator(int d, unsigned long nts)
    {
        delayMS = d;
        prevIterTimestamp = nts;
    };
};

namespace Iteration
{
    Iterator Line(36);
    Iterator Ping(50);
}

namespace Setup
{
    int botMode = 0; // Line, Line with obj detection, Fire detection, Water with fire detection, Remote Control
    int const LINE = 0;
    int const LINE_WITH_OBJ = 1;
    int const FIRE = 2;
    int const FIRE_WITH_WATER = 3;
    int const RC = 4;

    int innerSide = 0;
    int const LEFT = 0;
    int const RIGHT = 180;

    int const BUTTON_PIN = A0;
    unsigned long lastLeft = 0;

    Iterator Blink(0);
    int currentLEDState = LOW;
    unsigned int const DELAYS[5] = {0, 500, 1000, 3000, 50000};

}

namespace Motor
{
    int const Pins[2][3] = {{8, 7, 5}, {2, 4, 6}};
    int const Configs[3][2] = {{LOW, LOW}, {LOW, HIGH}, {HIGH, LOW}};

    int const STOP = 0;
    int const FORWARD = 1;
    int const BACKWARD = 2;

    char const MovementDirection[4] = {'F', 'R', 'L', 'S'};

    int const SPEED = 120;
    int const SLOW_SPEED = SPEED * (0.8);
    int const FAST_SPEED = 200;
    int const TURN_SPEED_DELTA = 50;

    int const SHARP_TURN_LIMIT = 10000;
}

namespace IR
{
    int const Pins[3] = {A2, A1};
}

namespace Ultrasonic
{
    int const TRIGGER = A3;
    int const ECHO = 13;
    int const MAX_DISTANCE = 30;

    NewPing sensor(TRIGGER, ECHO, MAX_DISTANCE);

    int const SERVO_PIN = 9;
    Servo servo;

    int const servoPositions[5] = {30, 60, 90, 120, 150};
    unsigned long readings[5] = {40, 40, 40, 40, 40};

    int posIndex = 3;
    int diff = 0;

    int checkForObject()
    {
        return (readings[posIndex] <= 30 && readings[posIndex] > 0);
    }

    void writeReading()
    {
        readings[posIndex] = sensor.ping_result / US_ROUNDTRIP_CM; // Ping returned, uS result in ping_result, convert to cm with US_ROUNDTRIP_CM.
    }

    void moveServo()
    {
        if (posIndex >= 5 || posIndex <= 0)
        {
            diff *= -1;
        }

        posIndex += diff;
        servo.write(servoPositions[posIndex]);
    }

} // namespace name

#endif