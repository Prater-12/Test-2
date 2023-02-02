#include <Arduino.h>
#include <Servo.h>
#include <NewPing.h>
#include <SoftwareSerial.h>

#ifndef MS_CONSTANTS
#define MS_CONSTANTS

int botMode = 0; // Line, Line with obj detection, Fire detection, Water with fire detection, Remote Control

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

    int checkCoolDown(unsigned long timestamp)
    {
        return timestamp - prevIterTimestamp > delayMS;
    }
};

namespace Iteration
{
    Iterator Line(36);
    Iterator Ping(50);
    Iterator Arm(10);
    Iterator FlameCheck(6000);
    Iterator FireDouse(5000);
}

namespace Setup
{
    int progress = 0;         // Progress into finishing setup, set to 2 to skip
    int waitUntilRelease = 0; // Only used when progress is set to 1

    int const PRESS_TIME = 100;
    int const HOLD_TIME = 2000;

    int const LINE = 0;
    int const LINE_WITH_OBJ = 1;
    int const FIRE = 2;
    int const FIRE_WITH_WATER = 3;
    int const RC = 4;

    int baseAngleIndex = 0;
    int const ARM_BASE_ANGLES[3] = {0, 90, 175};

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

    char const MovementDirection[5] = {'F', 'R', 'L', 'S', 'B'};

    int const SPEED = 150;
    int const SLOW_SPEED = 90;
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

    int const SERVO_PIN = 10;
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

namespace Arm
{
    int const BASE_SERVO_PIN = 9;
    Servo baseServo;

    int const END_SERVO_PIN = 10;
    Servo endServo;

    int const PUMP_RELAY = 12;

    int endPosition = 90;
    int diff = 1;

    int fireProbability = 0; // 0 to 2

    int const MAX_ANGLE = 135;
    int const MIN_ANGLE = 65;

    int const SensorPins[2] = {A4, A5}; // Lower,Upper
    int sensorReadings[2] = {1023, 1023};
}

#endif