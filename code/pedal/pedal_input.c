/*********************************************************************************************************************
* File: pedal_input.c
* Brief: 踏板 ADC 采样模块（第一步：A45/A47 原始值读取，4 点均值）
* Author: JX116
*********************************************************************************************************************/

#include "pedal_input.h"
#include "zf_driver_adc.h"

#define PEDAL_FILTER_COUNT  4U

static uint16 pedal_a45_raw = 0U;
static uint16 pedal_a47_raw = 0U;

void pedal_input_init(void)
{
    adc_init(ADC8_CH13_A45, ADC_12BIT);
    adc_init(ADC8_CH15_A47, ADC_12BIT);
}

void pedal_input_task(void)
{
    pedal_a45_raw = adc_mean_filter_convert(ADC8_CH13_A45, PEDAL_FILTER_COUNT);
    pedal_a47_raw = adc_mean_filter_convert(ADC8_CH15_A47, PEDAL_FILTER_COUNT);
}

uint16 pedal_input_get_a45(void)
{
    return pedal_a45_raw;
}

uint16 pedal_input_get_a47(void)
{
    return pedal_a47_raw;
}
