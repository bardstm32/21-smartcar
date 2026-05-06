#ifndef __VOLTAGE_WARNING_H__
#define __VOLTAGE_WARNING_H__

#include "zf_common_headfile.h"
#define ADC_CHANNEL1 (ADC_CH5_P15)

void voltage_warning_init(adc_channel_enum channel);

#endif
