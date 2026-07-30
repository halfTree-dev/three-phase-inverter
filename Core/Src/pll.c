#include "pll.h"
#include <math.h>

// SOGI-QSG 正交输出：v_alpha 同相，v_beta 滞后 90 度
static float v_alpha = 0.0f;
static float v_beta = 0.0f;

// 频率环 PI 积分项
static float pi_integral = 0.0f;

// 数控振荡器状态
static float theta = 0.0f;
static float omega = PLL_OMEGA_N;

// 缓存的交流输入电压瞬时值
static float v_in = 0.0f;

void PLL_Init(void) {
    v_alpha = 0.0f;
    v_beta = 0.0f;
    pi_integral = 0.0f;
    theta = 0.0f;
    omega = PLL_OMEGA_N;
    v_in = 0.0f;
}

// PLL 计算得到的 theta 将与电压的当前余弦相位相等，theta = omega t
void PLL_ProcessSample(uint32_t sample) {
    // 转换为交流电压
    float v_module = (float)sample * PLL_VREF / PLL_ADC_FULLSCALE;
    v_in = (v_module - PLL_MID_VOLT) * PLL_AC_SCALE;

    // SOGI-QSG 生成正交分量，其中 alpha 追踪 v_in，beta 对 alpha 积分落后 90 deg
    float err = v_in - v_alpha;
    float fwd = PLL_SOGI_K * err - v_beta;
    v_alpha += PLL_TS * PLL_OMEGA_N * fwd;
    v_beta += PLL_TS * PLL_OMEGA_N * v_alpha;

    // Park 变换将 alpha-beta 坐标转换为 dq 坐标
    float c = cosf(theta);
    float s = sinf(theta);
    float vq = -v_alpha * s + v_beta * c;

    // dq 坐标系上的 q 轴分量，作为当前角度和前一次角度的差值，输入到 PI 控制器锁定相位
    float pi_err = vq;
    float domega = PLL_KP * pi_err + PLL_KI * pi_integral;
    float domega_sat = domega;
    if (domega_sat > PLL_DOMEGA_MAX) {
        domega_sat = PLL_DOMEGA_MAX;
    } else if (domega_sat < -PLL_DOMEGA_MAX) {
        domega_sat = -PLL_DOMEGA_MAX;
    }
    if (domega_sat == domega) {
        pi_integral += pi_err * PLL_TS;
    }

    // PI 控制器的输出作为相位变化率，输出为角度增量
    omega = PLL_OMEGA_N + domega_sat;
    theta += omega * PLL_TS;
    if (theta >= PLL_TWO_PI) {
        theta -= PLL_TWO_PI;
    } else if (theta < 0.0f) {
        theta += PLL_TWO_PI;
    }
}

float PLL_GetTheta(void) {
    return theta;
}

float PLL_GetCosTheta(void) {
    return cosf(theta);
}

float PLL_GetSinTheta(void) {
    return sinf(theta);
}

float PLL_GetInputVoltage(void) {
    return v_in;
}

float PLL_GetFrequency(void) {
    return omega / PLL_TWO_PI;
}
