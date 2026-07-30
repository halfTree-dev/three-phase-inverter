#ifndef __INVERTER_H
#define __INVERTER_H

#include "main.h"

// 逆变器相位的分辨率，4096 对应 360 度
#define ANGLE_RESOLUTION 4096

// 调制比步进率
#define MODULATION_RATIO_MAX 0.90f
#define MODULATION_RATIO_MIN 0.05f
// 设置 20kHz / 1000 = 20Hz 的调制比更新循环
// 注意幅值也是每 20Hz 一更新
#define MODULATION_LOOP_COUNTER 1000

// 闭环目标参数
// 6V 的测试闭环目标参数
// #define TARGET_G_VPP 230.0f

// 32V 的测试闭环目标参数
#define TARGET_G_VPP 1780.0f

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

// 调制比调控的 PI 参数
#define MOD_CONTROL_KP 4.5e-5f
#define MOD_CONTROL_KI 1.5e-5f

// 初始化三相 SPWM 生成与交流采样闭环所需的外设（TIM1 PWM、注入 ADC）
void Inverter_Init();

// 每个 PWM 周期刷新三相 SPWM 比较值
void Inverter_UpdatePWM();

// 调制比闭环 PI 控制（每 MODULATION_LOOP_COUNTER 个周期结算一次）
void Inverter_UpdateModulation();

// 处理一次注入 ADC 采样（Goertzel 单频检测，满 N 点结算峰峰值）
void Inverter_ProcessSample(uint32_t sample);

#endif /* __INVERTER_H */
