#include <Arduino.h>
#ifndef MS_CONSTANTS
#define MS_CONSTANTS

int const mainEnable = 1;

namespace Iteration
{
    int const Line = 20;
}

namespace Motor
{
    int const Pins[2][3] = {{7, 6, 5}, {2, 4, 3}};
    int const Configs[3][2] = {{LOW, LOW}, {HIGH, LOW}, {LOW, HIGH}};

    int const STOP = 0;
    int const FORWARD = 1;
    int const BACKWARD = 2;

    char const wheelStates[4] = {'F', 'R', 'L', 'S'};

    int const SPEED = 150;
    int const TURN_SPEED = 90;
} // namespace name

namespace IR
{
    int const Pins[2] = {A2, A1};
}

#endif