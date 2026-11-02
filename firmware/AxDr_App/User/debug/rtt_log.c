/**
 * @file rtt_log.c
 * @brief RTT 日志薄封装实现。主链位置：无（只从 MAIN 上下文调用）。
 *
 * 格式化自实现（不用任何 printf 变体，遵守 00 规范 §6.2）。
 * 所有函数非阻塞：缓冲区满时丢弃本行，不等待 J-Link 读取。
 */

#include "rtt_log.h"

#include "SEGGER_RTT.h"

/* 一行日志的格式化缓冲；模块私有，只由 MAIN 上下文写。 */
static char line[64];

/* 把 "name: " 写入行首，返回已用长度。 */
static uint32_t line_reset(const char *name)
{
    uint32_t i = 0U;

    while ((name[i] != '\0') && (i < 30U))
    {
        line[i] = name[i];
        i++;
    }
    line[i++] = ':';
    line[i++] = ' ';
    line[i] = '\0';
    return i;
}

/* 把无符号整数按十进制追加到 line，返回追加后的长度。 */
static uint32_t append_u32(uint32_t len, uint32_t value)
{
    char digits[10];
    uint32_t n = 0U;
    uint32_t i;

    do
    {
        digits[n++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);

    for (i = n; i > 0U; i--)
    {
        line[len++] = digits[i - 1U];
    }
    line[len] = '\0';
    return len;
}

/* 定点小数：value 显示为 value/1000，保留 3 位小数。 */
static void append_q1000(uint32_t len, int32_t value)
{
    if (value < 0)
    {
        line[len++] = '-';
        value = -value;
    }
    len = append_u32(len, (uint32_t)value / 1000U);
    line[len++] = '.';
    line[len++] = (char)('0' + ((uint32_t)value % 1000U) / 100U);
    line[len++] = (char)('0' + ((uint32_t)value % 100U) / 10U);
    line[len++] = (char)('0' + ((uint32_t)value % 10U));
    line[len] = '\0';
}

void rtt_log_init(void)
{
    SEGGER_RTT_Init();
    SEGGER_RTT_WriteString(0, "AxDr RTT log\r\n");
}

void rtt_log_str(const char *s)
{
    SEGGER_RTT_WriteString(0, s);
    SEGGER_RTT_WriteString(0, "\r\n");
}

void rtt_log_u32(const char *name, uint32_t value)
{
    uint32_t len = line_reset(name);

    len = append_u32(len, value);
    rtt_log_str(line);
}

void rtt_log_f32(const char *name, float value)
{
    uint32_t len = line_reset(name);

    append_q1000(len, (int32_t)(value * 1000.0f));
    rtt_log_str(line);
}
