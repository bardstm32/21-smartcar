#include "zf_driver_exti.h"

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      失能全部 EXTI 中断
//  返回参数      void
//  使用示例      exti_all_close();
//-------------------------------------------------------------------------------------------------------------------
void exti_all_close (void)
{
	EX0 = 0;
	EX1 = 0;
	EX2 = 0;
	EX3 = 0;
	EX4 = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     EXTI 中断使能
// 参数说明     pin             选择 EXTI 引脚 (可选择范围由 zf_driver_exti.h 内 exti_pin_enum 枚举值确定)
// 返回参数     void
// 使用示例     exti_enable(INT0_P32);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void exti_enable (exti_pin_enum eru_pin)
{
	switch(eru_pin)
	{
		case INT0_P32: EX0 = 1; break;
		
		case INT1_P33: EX1 = 1; break;
		
		case INT2_P36: EX2 = 1; break;
		
		case INT3_P37: EX3 = 1; break;
		
		case INT4_P30: EX4 = 1; break;
		
		default: break;
	}
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     EXTI 中断失能
// 参数说明     pin             选择 EXTI 引脚 (可选择范围由 zf_driver_exti.h 内 exti_pin_enum 枚举值确定)
// 返回参数     void
// 使用示例     exti_disable(INT0_P32);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void exti_disable (exti_pin_enum eru_pin)
{
	switch(eru_pin)
	{
		case INT0_P32: EX0 = 0; break;
		
		case INT1_P33: EX1 = 0; break;
		
		case INT2_P36: EX2 = 0; break;
		
		case INT3_P37: EX3 = 0; break;
		
		case INT4_P30: EX4 = 0; break;
		
		default: break;
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      EXTI 中断初始化
//  参数说明      eru_pin         选择 EXTI 引脚 (可选择范围由 zf_driver_exti.h 内 exti_pin_enum 枚举值确定)
//  参数说明      trigger         设置触发方式
//  返回参数      void
//  使用示例      exti_init(INT0_P32, EXTI_TRIGGER_BOTH);  //初始化P32 作为外部中断引脚，双边沿触发。
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void exti_init (exti_pin_enum eru_pin, exti_trigger_enum trigger)
{
	gpio_init( eru_pin & 0XFF, GPIO, 1, GPIO_NO_PULL);
	
	// 配置触发方式（仅INT0/INT1可配置）
	switch(eru_pin)
    {
        case INT0_P32: 
			IT0 = trigger;
			break;
		
        case INT1_P33: 
			IT1 = trigger;
			break;
		
        case INT2_P36: 
			 // 仅下降沿触发，无需配置
			break;
		
        case INT3_P37: 
			 // 仅下降沿触发，无需配置
			break;
		
        case INT4_P30: 
			 // 仅下降沿触发，无需配置
			break;
		
        default: break;
    }

	exti_enable(eru_pin);	// 中断使能
}

