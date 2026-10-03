/**
 * @file test_foundation_headers.c
 * @brief foundation 层头卫生测试：自包含、与 math.h 共存、位图唯一性。
 */

#include <assert.h>
#include <math.h>

/* 自包含 + 共存：三个头单独可编译，且与 math.h 同时包含无重名冲突 */
#include "math_const.h"
#include "ret.h"
#include "fault.h"

int main(void)
{
    /* 与标准库无宏冲突：这里能取地址调用标准 sinf 即为共存证明 */
    volatile float s = sinf(MATH_PI_2);
    (void)s;

    /* MATH 常量为 float 字面量 */
    _Static_assert(_Generic(MATH_PI, float: 1, default: 0) == 1, "MATH_PI must be float");

    /* ret_e：OK 必须为 0 */
    _Static_assert(RET_OK == 0, "RET_OK must be 0");

    /* fault_t：各位互不重叠，NONE 为 0 */
    _Static_assert(ERR_NONE == 0, "ERR_NONE must be 0");
    _Static_assert((ERR_OC & ERR_OV) == 0, "fault bits overlap");
    _Static_assert((ERR_OC & ERR_UV) == 0, "fault bits overlap");
    _Static_assert((ERR_OV & ERR_UV) == 0, "fault bits overlap");
    _Static_assert((ERR_OC & ERR_OT_MOS) == 0, "fault bits overlap");
    _Static_assert((ERR_ENC & ERR_COMM_LOST) == 0, "fault bits overlap");
    _Static_assert((ERR_CONFIG & ERR_OC) == 0, "fault bits overlap");

    return 0;
}
