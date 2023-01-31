#include <Arduino.h>
#ifndef MS_CONSTANTS
#define MS_CONSTANTS

int const mainEnable = 1;

namespace Iteration
{
    int const Line = 72;
}

namespace Motor
{
    int const Pins[2][3] = {{7, 6, 5}, {2, 4, 3}};
    int const Configs[3][2] = {{LOW, LOW}, {LOW, HIGH}, {HIGH, LOW}};

    int const STOP = 0;
    int const FORWARD = 1;
    int const BACKWARD = 2;

    char const MovementDirection[4] = {'F', 'R', 'L', 'S'};

    int const SPEED = 120;
    int const SLOW_SPEED = SPEED * (0.8);
    int const FAST_SPEED = 200;
    int const TURN_SPEED_DELTA = 20;

    int const TURN_LIMIT = 5;
}

namespace IR
{
    int const Pins[3] = {A2, A1, A0};
}

#endif