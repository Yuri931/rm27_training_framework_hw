#ifndef CONTROLTHREAD_HPP
#define CONTROLTHREAD_HPP
#include "pid.hpp"

namespace ControlThread
{
    void SetSpeedTarget(float rad_per_s);
    void SetPositionTarget(float rad);
    void SetEnable(bool on);
    bool IsFault();
}

#endif