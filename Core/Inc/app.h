#ifndef __APP_H
#define __APP_H

#include "main.h"

// 逆变器相位的分辨率，4096 对应 360 度
#define ANGLE_RESOLUTION 4096

// 调制比步进率
#define MODULATION_RATIO_STEP 0.0001f
#define MODULATION_RATIO_MAX 0.95f
#define MODULATION_RATIO_MIN 0.05f
// 设置 20kHz / 1000 = 20Hz 的调制比更新循环
// 注意幅值也是每 20Hz 一更新
#define MODULATION_LOOP_COUNTER 1000

// 闭环目标参数
// 5V 的测试闭环目标参数
// #define TARGET_G_VPP 195.0f

// 32V 的测试闭环目标参数
#define TARGET_G_VPP 1750.0f

#define TARGET_G_VPP_DEADZONE 10.0f

// 计数器
#define COUNTER_PERIOD 8000.0f

#define COUNTER_INCREASE_50Hz 10737418
#define COUNTER_INCREASE_60Hz 12884902

#define ONE_OF_THREE_UINT32_MAX 1431655765U
#define TWO_OF_THREE_UINT32_MAX 2863311530U
#define DOUBLE_PI 6.2831853f

// ADC 注入采样与单频检测参数
// 采样率 fs = 20 kHz（TIM1 周期 8000 @160MHz，CC4 每周期触发一次注入转换）
// 信号基频 f0 = 60 Hz
// 取 N = 1000 点恰好覆盖 3 个 60Hz 周期（50 ms），对应 DFT bin k = 3（整数），实现相干采样
#define SAMPLE_RATE_HZ 20000u
#define SIG_FREQ_HZ    60u
#define GOERTZEL_N     1000u
#define GOERTZEL_K     3u
// Goertzel 系数 coeff = 2*cos(2*pi*k/N) = 2*cos(2*pi*3/1000)
#define GOERTZEL_COEFF 1.9996447f

void App_Init();
void App_Loop();

#endif /* __APP_H */