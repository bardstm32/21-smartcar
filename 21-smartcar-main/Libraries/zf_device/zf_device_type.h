#ifndef _zf_device_type_h_
#define _zf_device_type_h_

#include "zf_common_typedef.h"
#include "zf_driver_uart.h"

typedef enum
{
    NO_WIRELESS = 0,                                                            // 无设备
    WIRELESS_UART,                                                              // 无线串口
    BLE6A20,																	// 蓝牙BLE6A20
	LORA3A22_UART																// LORA
} wireless_type_enum;

typedef enum
{
    NO_TOF = 0,                                                                 // 无设备
    TOF_DL1A,                                                                   // DL1A
    TOF_DL1B,                                                                   // DL1B
} tof_type_enum;

typedef enum
{
    NO_GPS = 0,                                                                 // 无设备
    GPS_TAU1201,                                                                   // DL1A

} gps_type_enum;


#define	SOFT_IIC 		(0)
#define SOFT_SPI 		(1)                                                              // 无设备
#define	HARDWARE_IIC 	(2)
#define	HARDWARE_SPI 	(3)


extern gps_type_enum 		gps_type;
extern tof_type_enum 		tof_type;
extern wireless_type_enum  	wireless_type;

extern void (*wireless_module_uart_handler)	(uint8 dat);
extern void (*tof_module_exti_handler)		(uint8 dat);


extern void set_wireless_type (wireless_type_enum type_set, uart_index_enum uartx, void *uart_callback);
extern void set_tof_type (tof_type_enum type_set, void *exti_callback);
extern void set_gps_type (gps_type_enum type_set, uart_index_enum uartx, void *uart_callback);

#endif
