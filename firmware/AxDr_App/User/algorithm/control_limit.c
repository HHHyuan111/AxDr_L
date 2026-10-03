/**
 * @file control_limit.c
 * @brief 与硬件无关的控制量限幅实现。
 */

#include "control_limit.h"

float control_limit(float value, float upper, float lower)
{
    if (value > upper)
    {
        return upper;
    }
    else if (value < lower)
    {
        return lower;
    }
    else
    {
        return value;
    }
}
