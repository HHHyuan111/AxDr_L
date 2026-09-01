#include "mc_common.h"

#include <math.h>
#include <string.h>

bool mc_float_is_finite(float value)
{
    uint32_t bits = 0u;
    typedef char mc_float_must_be_binary32[
        (sizeof(float) == sizeof(bits)) ? 1 : -1];
    (void)sizeof(mc_float_must_be_binary32);
    memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT32_C(0x7F800000)) != UINT32_C(0x7F800000);
}

float mc_clampf(float value, float lower, float upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

float mc_wrap_0_2pi(float angle)
{
    if (!mc_float_is_finite(angle)) return 0.0f;
    angle = fmodf(angle, MC_TWO_PI_F);
    if (angle < 0.0f) angle += MC_TWO_PI_F;
    return angle;
}

float mc_wrap_pm_pi(float angle)
{
    if (!mc_float_is_finite(angle)) return 0.0f;
    angle = fmodf(angle + MC_PI_F, MC_TWO_PI_F);
    if (angle < 0.0f) angle += MC_TWO_PI_F;
    return angle - MC_PI_F;
}




