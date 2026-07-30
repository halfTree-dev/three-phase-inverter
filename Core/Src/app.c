#include "app.h"
#include "inverter.h"
#include "pll.h"

void App_Init() {
    HAL_Delay(1000);
    Inverter_Init();
    PLL_Init();
}

void App_Loop() {
    HAL_GPIO_TogglePin(LED_PIN_OUT_GPIO_Port, LED_PIN_OUT_Pin);
    HAL_Delay(100);
}

// TIM1 中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
        // 刷新三相 SPWM 并执行调制比闭环
        Inverter_UpdatePWM();
        Inverter_UpdateModulation();
    }
}

// ADC 注入完成回调函数
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    // 读取反馈电压并送入 Goertzel 单频检测
    uint32_t inverter_sample = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    Inverter_ProcessSample(inverter_sample);

    // 交流输入电压采样模块的读数，送入 SOGI-PLL 推进相位
    uint32_t v_in_sample = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    PLL_ProcessSample(v_in_sample);

    volatile uint32_t sample3 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
    volatile uint32_t sample4 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4);
}