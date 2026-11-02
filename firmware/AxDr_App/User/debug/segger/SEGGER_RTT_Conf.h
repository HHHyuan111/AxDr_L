/*********************************************************************
*        SEGGER RTT * Real Time Transfer for embedded targets        *
**********************************************************************

---------------------------END-OF-HEADER------------------------------
Purpose : AxDr 的 RTT 配置。日志只从 MAIN 上下文写，缓冲区按需收敛。
----------------------------------------------------------------------
*/

#ifndef SEGGER_RTT_CONF_H
#define SEGGER_RTT_CONF_H

#define BUFFER_SIZE_UP                            (512)
#define BUFFER_SIZE_DOWN                          (16)
#define SEGGER_RTT_MAX_NUM_UP_BUFFERS             (2)

#endif  /* SEGGER_RTT_CONF_H */
