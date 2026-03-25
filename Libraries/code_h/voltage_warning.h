#ifndef __VOLTAGE_WARNING_H__
#define __VOLTAGE_WARNING_H__

#include "zf_common_headfile.h"
#define Battery_Channel (ADC_CH5_P15)

void voltage_warning_init(void);
void voltage_warning_task(float target_voltage);

#endif
