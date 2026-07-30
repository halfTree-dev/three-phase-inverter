#include "app.h"
#include "inverter.h"

void App_Init() {
    HAL_Delay(1000);
    Inverter_Init();
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
    volatile uint32_t sample = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    Inverter_ProcessSample(sample);

    volatile uint32_t sample2 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    volatile uint32_t sample3 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
    volatile uint32_t sample4 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4);
}