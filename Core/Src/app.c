#include "app.h"
#include <stdio.h>
#include <math.h>

extern ADC_HandleTypeDef hadc1;

extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim1;

extern UART_HandleTypeDef huart1;

// 采样结果和幅值推断结果
volatile uint32_t adc_value;
volatile float g_vpp_code = 0.0f;
volatile float g_vpp_sum = 0.0f;
volatile float g_vpp_avg = 0.0f;

// 调制比
float modulation_ratio = 0.2f;

uint16_t _Get_SPWM_Counter_Value(uint32_t current_phase) {
    return (uint16_t)((sinf((float)current_phase * DOUBLE_PI / 4294967296.0f) * modulation_ratio * 0.5f + 0.5f) * COUNTER_PERIOD);
}

uint32_t current_phase = 0;

void App_Init() {
    volatile float dummy = sinf(0.5f);
    HAL_Delay(1000);

    HAL_TIM_Base_Start_IT(&htim1);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    LL_ADC_StartCalibration(ADC1, ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC1));

    HAL_ADCEx_InjectedStart_IT(&hadc1);
}

void App_Loop() {
    HAL_GPIO_TogglePin(LED_PIN_OUT_GPIO_Port, LED_PIN_OUT_Pin);
    HAL_Delay(100);
}


// TIM1 中断回调函数，该函数将根据当前电机状态控制 PWM 占空比
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    // 调制比更新 COUNTER
    static uint32_t modulation_loop_counter = 0u;

    if (htim->Instance == TIM1) {
        // 更新电机状态并设置新的占空比值
        current_phase += COUNTER_INCREASE_60Hz;
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, _Get_SPWM_Counter_Value(current_phase));
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, _Get_SPWM_Counter_Value(current_phase + ONE_OF_THREE_UINT32_MAX));
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, _Get_SPWM_Counter_Value(current_phase + TWO_OF_THREE_UINT32_MAX));
        g_vpp_sum += g_vpp_code;

        // 调制比受到 PID 调控
        if (++modulation_loop_counter >= MODULATION_LOOP_COUNTER) {
            static float last_err = 0.0f;

            g_vpp_avg = g_vpp_sum / MODULATION_LOOP_COUNTER;

            float modulation_ratio_delta = 0.0f;
            float err = TARGET_G_VPP - g_vpp_avg;
            modulation_ratio_delta = MOD_CONTROL_KP * (err - last_err) + MOD_CONTROL_KI * err;
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
    }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    // Goertzel 单频检测递推状态（在 ADC 注入完成回调中逐样更新）
    static float g_s1 = 0.0f;
    static float g_s2 = 0.0f;
    static uint32_t g_n = 0u;

    // 读取得到的反馈电压数据
    adc_value = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);

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