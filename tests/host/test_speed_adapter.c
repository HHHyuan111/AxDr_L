/** 连续窗口测速边界：初始化、丢样、跨零、序号回绕与多实例。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "speed_adapter.h"
#include "math_const.h"

static void near(float got, float want) { assert(fabsf(got - want) < 0.003f); }
int main(void)
{
    speed_est_t a, b;
    speed_est_init(&a); speed_est_init(&b);
    speed_est_step(&a, 1.0f, 20000.0f, 10U, true);
    speed_est_step(&a, 1.0f, 20000.0f, 11U, true);
    near(speed_est_get(&a), 0.0f); /* 静止非零角不能和未填充的 0 做差。 */
    speed_est_step(&a, 1.0f, 20000.0f, 12U, true);
    near(speed_est_get(&a), 0.0f);

    speed_est_init(&a);
    for (unsigned i=0; i<5; ++i)
    {
        speed_est_step(&a, 1.0f + 0.001f*i, 20000.0f, i, true);
        speed_est_step(&b, 2.0f - 0.002f*i, 20000.0f, i, true);
    }
    near(speed_est_get(&a),20.0f); near(speed_est_get(&b),-40.0f);
    speed_est_step(&a, NAN, 20000.0f, 5U, false);
    for (unsigned i=6; i<9; ++i)
    {
        speed_est_step(&a, 1.0f + 0.001f*i, 20000.0f, i, true);
        near(speed_est_get(&a),20.0f); /* 不得产生旧实现的 30 rad/s。 */
    }
    /* 未调用期间缺了一拍，同样重建；暂停很久不跨空窗做差。 */
    speed_est_step(&a, 2.0f, 20000.0f, 100U, true);
    near(speed_est_get(&a),20.0f);
    speed_est_step(&a, 2.0f, 20000.0f, 101U, true);
    near(speed_est_get(&a),20.0f);
    speed_est_step(&a, 2.0f, 20000.0f, 102U, true);
    near(speed_est_get(&a),0.0f);

    speed_est_init(&a);
    speed_est_step(&a, 6.2820f, 20000.0f, UINT32_MAX-1U, true);
    speed_est_step(&a, 0.0005f, 20000.0f, UINT32_MAX, true);
    speed_est_step(&a, 0.0015f, 20000.0f, 0U, true);
    near(speed_est_get(&a),(0.0015f-6.2820f+MATH_2PI)*10000.0f);
    const float before = speed_est_get(&a);
    speed_est_step(&a, NAN, 20000.0f, 1U, true);
    near(speed_est_get(&a), before);
    speed_est_step(&a, 1.0f, 0.0f, 2U, true);
    near(speed_est_get(&a), before);
    speed_est_init(&a);
    near(speed_est_get(&a),0.0f); near(speed_est_get(&b),-40.0f);
    puts("speed adapter: PASS (warmup, gaps, wrap, invalid samples, reset, independent motors)");
    return 0;
}
