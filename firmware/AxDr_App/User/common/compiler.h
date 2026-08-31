/**
 * @file compiler.h
 * @brief 项目自有代码使用的编译器属性适配。
 */

#ifndef PLATFORM_COMPILER_H
#define PLATFORM_COMPILER_H

/*
 * 目标固件把快速函数放入现有 .RamFunc 段。Host 或其他编译器不支持该属性时，
 * 宏退化为空，算法接口和数值行为保持不变。
 */
#if defined(__GNUC__) || defined(__clang__) || defined(__CC_ARM)
#define PLATFORM_FAST_CODE __attribute__((section(".RamFunc")))
#define PLATFORM_FAST_DATA __attribute__((section(".data")))
#else
#define PLATFORM_FAST_CODE
#define PLATFORM_FAST_DATA
#endif

#endif /* PLATFORM_COMPILER_H */
