#ifndef __PLL_H
#define __PLL_H

#include "main.h"

// 注入 ADC 采样率 20kHz（TIM1 每周期触发一次注入转换）
#define PLL_SAMPLE_RATE_HZ  20000u
#define PLL_TS              (1.0f / (float)PLL_SAMPLE_RATE_HZ)

// 市电基频 50Hz
#define PLL_GRID_FREQ_HZ    50u
#define PLL_TWO_PI          6.2831853f
#define PLL_OMEGA_N         (PLL_TWO_PI * (float)PLL_GRID_FREQ_HZ)

// ADC 量纲转换：采样模块 AC 0V -> 1.65V，AC [-55,55]V -> [0,3.3]V
#define PLL_ADC_FULLSCALE   4095.0f
#define PLL_VREF            3.3f
#define PLL_MID_VOLT        1.65f
#define PLL_AC_FULLSCALE    55.0f
// AC 电压换算系数 = AC_FULLSCALE / MID_VOLT = 55 / 1.65
#define PLL_AC_SCALE        (PLL_AC_FULLSCALE / PLL_MID_VOLT)

// SOGI-QSG 阻尼系数，最优取 sqrt(2) ≈ 1.4142
#define PLL_SOGI_K          1.4142f

// 频率环 PI 参数（驱动 vq -> 0）
// 按环路自然频率约 20Hz、阻尼比 0.707 设计：Kp = 2*zeta*wn，Ki = wn^2
#define PLL_KP              180.0f
#define PLL_KI              15000.0f
// 允许的角频率偏差限幅（对应约 +-10Hz），用于抗积分饱和
#define PLL_DOMEGA_MAX      (PLL_TWO_PI * 10.0f)

// 初始化锁相环内部状态
void PLL_Init(void);

// 喂入一次输入电压通道的 ADC 原始码值，推进 SOGI-PLL 一步
void PLL_ProcessSample(uint32_t sample);

// 读取最近一次估算的相位角 theta（弧度，[0, 2*pi)）
float PLL_GetTheta(void);

// 读取 cos(theta) / sin(theta)
float PLL_GetCosTheta(void);
float PLL_GetSinTheta(void);

// 读取换算后的交流输入电压瞬时值（伏，已去直流偏置）
float PLL_GetInputVoltage(void);

// 读取锁相环估算的电网频率（Hz）
float PLL_GetFrequency(void);

#endif /* __PLL_H */
