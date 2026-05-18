

#include "zf_common_headfile.h"

void INT0_IRQHandler() interrupt 0
{
	INT0_CLEAR_FLAG;
	if (int0_irq_handler != NULL)
    {
        int0_irq_handler();
    }
}

void INT1_IRQHandler(void) interrupt 2
{
	INT1_CLEAR_FLAG;
	if (int1_irq_handler != NULL)
    {
		int1_irq_handler();
    }
}

void INT2_IRQHandler(void) interrupt 10
{
	INT2_CLEAR_FLAG;
	if (int2_irq_handler != NULL)
    {
		int2_irq_handler();
    }
}
void INT3_IRQHandler(void) interrupt 11
{
	INT3_CLEAR_FLAG;
	if (int3_irq_handler != NULL)
    {
		int3_irq_handler();
    }
}

void INT4_IRQHandler(void) interrupt 16
{
	INT4_CLEAR_FLAG;
	if (int4_irq_handler != NULL)
    {
		int4_irq_handler();
    }
}

void DMA_UART1_IRQHandler(void) interrupt 4
{
    static vuint8 dwon_count = 0;

    if (DMA_UR1R_STA & 0x01)
    {
        DMA_UR1R_STA &= ~0x01;
        uart_rx_start_buff(UART_1);

        if (uart_rx_buff[UART_1][0] == 0x7F)
        {
            if (dwon_count++ > 20)
            {
                IAP_CONTR = 0x60;
            }
        }
        else
        {
            dwon_count = 0;
        }

        if (uart1_irq_handler != NULL)
        {
            uart1_irq_handler(uart_rx_buff[UART_1][0]);
        }
    }

    if (DMA_UR1R_STA & 0x02)
    {
        DMA_UR1R_STA &= ~0x02;
        uart_rx_start_buff(UART_1);


    }
}

void DMA_UART2_IRQHandler(void) interrupt 8
{
    if (DMA_UR2R_STA & 0x01)
    {
        DMA_UR2R_STA &= ~0x01;
        uart_rx_start_buff(UART_2);

        if (uart2_irq_handler != NULL)
        {
            uart2_irq_handler(uart_rx_buff[UART_2][0]);
        }
    }

    if (DMA_UR2R_STA & 0x02)
    {
        DMA_UR2R_STA &= ~0x02;
        uart_rx_start_buff(UART_2);


    }
}

void DMA_UART3_IRQHandler(void) interrupt 17
{
    if (DMA_UR3R_STA & 0x01)
    {
        DMA_UR3R_STA &= ~0x01;
        uart_rx_start_buff(UART_3);

        if (uart3_irq_handler != NULL)
        {
            uart3_irq_handler(uart_rx_buff[UART_3][0]);
        }
    }

    if (DMA_UR3R_STA & 0x02)
    {
        DMA_UR3R_STA &= ~0x02;
        uart_rx_start_buff(UART_3);


    }
}

void DMA_UART4_IRQHandler(void) interrupt 18
{
    if (DMA_UR4R_STA & 0x01)
    {
        DMA_UR4R_STA &= ~0x01;
        uart_rx_start_buff(UART_4);

        if (uart4_irq_handler != NULL)
        {
            uart4_irq_handler(uart_rx_buff[UART_4][0]);
        }
    }

    if (DMA_UR4R_STA & 0x02)
    {
        DMA_UR4R_STA &= ~0x02;
        uart_rx_start_buff(UART_4);


    }
}

void TM0_IRQHandler() interrupt 1
{
    TIM0_CLEAR_FLAG;

    if (tim0_irq_handler != NULL)
    {
        tim0_irq_handler();
    }
}

void TM1_IRQHandler() interrupt 3
{
    TIM1_CLEAR_FLAG;

    if (tim1_irq_handler != NULL)
    {
        tim1_irq_handler();
    }
}

void TM2_IRQHandler() interrupt 12
{
    TIM2_CLEAR_FLAG;

    if (tim2_irq_handler != NULL)
    {
        tim2_irq_handler();
    }
}

void TM3_IRQHandler() interrupt 19
{
    TIM3_CLEAR_FLAG;

    if (tim3_irq_handler != NULL)
    {
        tim3_irq_handler();
    }
}

void TM4_IRQHandler() interrupt 20
{
    TIM4_CLEAR_FLAG;

    if (tim4_irq_handler != NULL)
    {
        tim4_irq_handler();
    }
}

