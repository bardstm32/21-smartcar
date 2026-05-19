#ifndef _zf_device_wireless_uart_h_
#define _zf_device_wireless_uart_h_

#include "zf_common_typedef.h"

#define WIRELESS_UART_INDEX        ( UART_4 )          		// 无线串口对应使用的串口号
#define WIRELESS_UART_BUAD_RATE    ( 115200 )          		// 无线串口对应使用的串口波特率
#define WIRELESS_UART_TX_PIN       ( UART4_RX_P02 )    		// 无线串口对应模块的 TX 要接到单片机的 RX
#define WIRELESS_UART_RX_PIN       ( UART4_TX_P03 )    		// 无线串口对应模块的 RX 要接到单片机的 TX
#define WIRELESS_UART_RTS_PIN      ( IO_P07 )          		// 无线串口对应模块的 RTS 引脚


// ------------------------------------ 自动波特率 ------------------------------------
// 注意事项1：无线转串口模块版本是V2.0以下的是无法开启自动波特率的。
// 注意事项2：开启自动波特率务必连接RTS引脚 否则会开启失败。
// 注意事项3：模块自动波特率失败的话 可以尝试断电重启

// 开启自动波特率务必阅读上面两条 注意事项
// 开启自动波特率务必阅读上面两条 注意事项
// 开启自动波特率务必阅读上面两条 注意事项

// 0：关闭自动波特率
// 1：开启自动波特率 自动波特率的作用是修改 WIRELESS_UART_BAUD 之后不需要对模块进行配置 模块会自动设置为对应的波特率

#define WIRELESS_UART_AUTO_BAUD_RATE    ( 0 )

#if (1 == WIRELESS_UART_AUTO_BAUD_RATE)
typedef enum
{
    WIRELESS_UART_AUTO_BAUD_RATE_SUCCESS,
    WIRELESS_UART_AUTO_BAUD_RATE_INIT,
    WIRELESS_UART_AUTO_BAUD_RATE_START,
    WIRELESS_UART_AUTO_BAUD_RATE_GET_ACK,
} wireless_uart_auto_baudrate_state_enum;
#endif
// ------------------------------------ 自动波特率 ------------------------------------

#define WIRELESS_UART_BUFFER_SIZE       ( 64   )
#define WIRELESS_UART_TIMEOUT_COUNT     ( 0x64 )

uint32      wireless_uart_send_byte         (const uint8 dat);
uint32      wireless_uart_send_buffer       (const uint8 *buff, uint32 len);
uint32      wireless_uart_send_string       (const char *str);

uint32      wireless_uart_read_buffer       (uint8 *buff, uint32 len);

void        wireless_uart_callback          (uint8 uart_dat);

uint8       wireless_uart_init              (void);

#endif
