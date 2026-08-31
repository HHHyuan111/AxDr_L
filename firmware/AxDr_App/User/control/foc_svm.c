/**
 * @file foc_svm.c
 * @brief 六扇区空间矢量调制的无状态数值实现。
 */

#include "foc_svm.h"

#include "compiler.h"

static const float foc_one_by_sqrt3 = 0.57735026919f;
static const float foc_two_by_sqrt3 = 1.15470053838f;

PLATFORM_FAST_CODE int foc_svm(float v_alpha_norm,
                             float v_beta_norm,
                             float *duty_a,
                             float *duty_b,
                             float *duty_c)
{
    int sextant;
    float phase_a;
    float phase_b;
    float phase_c;

    if (v_beta_norm >= 0.0f)
    {
        if (v_alpha_norm >= 0.0f)
        {
            if (foc_one_by_sqrt3 * v_beta_norm > v_alpha_norm)
            {
                sextant = 2;
            }
            else
            {
                sextant = 1;
            }
        }
        else
        {
            if (-foc_one_by_sqrt3 * v_beta_norm > v_alpha_norm)
            {
                sextant = 3;
            }
            else
            {
                sextant = 2;
            }
        }
    }
    else
    {
        if (v_alpha_norm >= 0.0f)
        {
            if (-foc_one_by_sqrt3 * v_beta_norm > v_alpha_norm)
            {
                sextant = 5;
            }
            else
            {
                sextant = 6;
            }
        }
        else
        {
            if (foc_one_by_sqrt3 * v_beta_norm > v_alpha_norm)
            {
                sextant = 4;
            }
            else
            {
                sextant = 5;
            }
        }
    }

    switch (sextant)
    {
        case 1:
        {
            const float t1 = v_alpha_norm - foc_one_by_sqrt3 * v_beta_norm;
            const float t2 = foc_two_by_sqrt3 * v_beta_norm;

            phase_a = (1.0f - t1 - t2) * 0.5f;
            phase_b = phase_a + t1;
            phase_c = phase_b + t2;
            break;
        }

        case 2:
        {
            const float t2 = v_alpha_norm + foc_one_by_sqrt3 * v_beta_norm;
            const float t3 = -v_alpha_norm + foc_one_by_sqrt3 * v_beta_norm;

            phase_b = (1.0f - t2 - t3) * 0.5f;
            phase_a = phase_b + t3;
            phase_c = phase_a + t2;
            break;
        }

        case 3:
        {
            const float t3 = foc_two_by_sqrt3 * v_beta_norm;
            const float t4 = -v_alpha_norm - foc_one_by_sqrt3 * v_beta_norm;

            phase_b = (1.0f - t3 - t4) * 0.5f;
            phase_c = phase_b + t3;
            phase_a = phase_c + t4;
            break;
        }

        case 4:
        {
            const float t4 = -v_alpha_norm + foc_one_by_sqrt3 * v_beta_norm;
            const float t5 = -foc_two_by_sqrt3 * v_beta_norm;

            phase_c = (1.0f - t4 - t5) * 0.5f;
            phase_b = phase_c + t5;
            phase_a = phase_b + t4;
            break;
        }

        case 5:
        {
            const float t5 = -v_alpha_norm - foc_one_by_sqrt3 * v_beta_norm;
            const float t6 = v_alpha_norm - foc_one_by_sqrt3 * v_beta_norm;

            phase_c = (1.0f - t5 - t6) * 0.5f;
            phase_a = phase_c + t5;
            phase_b = phase_a + t6;
            break;
        }

        case 6:
        default:
        {
            const float t6 = -foc_two_by_sqrt3 * v_beta_norm;
            const float t1 = v_alpha_norm + foc_one_by_sqrt3 * v_beta_norm;

            phase_a = (1.0f - t6 - t1) * 0.5f;
            phase_c = phase_a + t1;
            phase_b = phase_c + t6;
            break;
        }
    }

    *duty_a = phase_a;
    *duty_b = phase_b;
    *duty_c = phase_c;

    if ((*duty_a >= 0.0f) && (*duty_a <= 1.0f) &&
        (*duty_b >= 0.0f) && (*duty_b <= 1.0f) &&
        (*duty_c >= 0.0f) && (*duty_c <= 1.0f))
    {
        return 0;
    }

    return -1;
}
