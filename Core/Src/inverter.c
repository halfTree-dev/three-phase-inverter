#include "inverter.h"
#include <math.h>

extern ADC_HandleTypeDef hadc1;

extern TIM_HandleTypeDef htim1;

// 调制比
static float modulation_ratio = 0.2f;

// 三相相位累加器
static uint32_t current_phase = 0u;

// 调制比 PI 控制的周期计数器与上一次误差
static uint32_t modulation_loop_counter = 0u;
static float last_err = 0.0f;

// 采样结果与幅值推断结果
static volatile uint32_t adc_value;
static volatile float g_vpp_code = 0.0f;
static volatile float g_vpp_sum = 0.0f;
static volatile float g_vpp_avg = 0.0f;

// Goertzel 单频检测递推状态
static float g_s1 = 0.0f;
static float g_s2 = 0.0f;
static uint32_t g_n = 0u;

// 根据相位计算单相 SPWM 比较值
static uint16_t Get_SPWM_Counter_Value(uint32_t phase) {
    return (uint16_t)((sinf((float)phase * DOUBLE_PI / 4294967296.0f) * modulation_ratio * 0.5f + 0.5f) * COUNTER_PERIOD);
}

void Inverter_Init() {
    volatile float dummy = sinf(0.5f);
    (void)dummy;

    HAL_TIM_Base_Start_IT(&htim1);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    LL_ADC_StartCalibration(ADC1, ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC1));

    HAL_ADCEx_InjectedStart_IT(&hadc1);
}

// 每个 PWM 周期调用：推进相位并刷新三相比较值
void Inverter_UpdatePWM() {
    current_phase += COUNTER_INCREASE_60Hz;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Get_SPWM_Counter_Value(current_phase));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Get_SPWM_Counter_Value(current_phase + ONE_OF_THREE_UINT32_MAX));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Get_SPWM_Counter_Value(current_phase + TWO_OF_THREE_UINT32_MAX));
}

// 调制比闭环 PI 控制：累加采样幅值，每 MODULATION_LOOP_COUNTER 个周期结算一次
void Inverter_UpdateModulation() {
    g_vpp_sum += g_vpp_code;

    if (++modulation_loop_counter < MODULATION_LOOP_COUNTER) {
        return;
    }

    g_vpp_avg = g_vpp_sum / MODULATION_LOOP_COUNTER;

    float err = TARGET_G_VPP - g_vpp_avg;
    float modulation_ratio_delta = MOD_CONTROL_KP * (err - last_err) + MOD_CONTROL_KI * err;

    modulation_loop_counter = 0u;
    g_vpp_sum = 0.0f;

    if (modulation_ratio + modulation_ratio_delta > MODULATION_RATIO_MAX) {
        modulation_ratio = MODULATION_RATIO_MAX;
    } else if (modulation_ratio + modulation_ratio_delta < MODULATION_RATIO_MIN) {
        modulation_ratio = MODULATION_RATIO_MIN;
    } else {
        modulation_ratio += modulation_ratio_delta;
    }

    last_err = err;
}

// 处理一次 ADC 注入采样：Goertzel 递推，满 N 点结算一次峰峰值并复位状态
void Inverter_ProcessSample(uint32_t sample) {
    adc_value = sample;

    // Goertzel 递推：每个采样更新一次谐振器状态
    float x = (float)adc_value;
    float s = x + GOERTZEL_COEFF * g_s1 - g_s2;
    g_s2 = g_s1;
    g_s1 = s;

    // 满 N 点后结算一次峰峰值，并复位状态开始下一窗（每 50 ms 刷新一次）
    if (++g_n >= GOERTZEL_N) {
        float mag2 = g_s1 * g_s1 + g_s2 * g_s2 - GOERTZEL_COEFF * g_s1 * g_s2;
        if (mag2 < 0.0f) {
            mag2 = 0.0f;
        }
        // 单频幅度 A = 2*sqrt(mag2)/N；峰峰值 Vpp = 2*A = 4*sqrt(mag2)/N
        g_vpp_code = 4.0f * sqrtf(mag2) / (float)GOERTZEL_N;
        g_n = 0u;
        g_s1 = 0.0f;
        g_s2 = 0.0f;
    }
}
