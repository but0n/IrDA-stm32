#include "stm32f10x.h"
#include "uart.h"
#include "irda.h"
#define NVIC_GROUPING	3

void delay(volatile unsigned int count) {
    for(count *= 12000; count!=0; count--);
}

int main() {
	NVIC_SetPriorityGrouping(0x07 - NVIC_GROUPING);
	// 设置中断优先级分组为3, 在寄存器中: xxx.y (x 指抢占式优先级, y 指响应式优先级)

	uart_init(72, 115200);
	irda_init();
	while(1) {
		uart_num2char(g_IrDA_Device[0].token[0]);
		uart_sendStr(", ");
		uart_num2char(g_IrDA_Device[0].token[1]);
		uart_sendStr(", ");
		uart_num2char(g_IrDA_Device[0].token[2]);
		uart_sendStr(", ");
		uart_num2char(g_IrDA_Device[0].token[3]);
		uart_sendStr(", ");
		uart_num2char(g_IrDA_Device[0].token[4]);
		uart_sendStr(", ");
		uart_num2char(g_IrDA_Device[0].token[5]);
		uart_sendStr(", ");
		uart_num2char(g_IrDA_Device[0].token[6]);
		uart_sendStr(", ");
		uart_num2char(g_IrDA_Device[0].token[7]);
		uart_sendStr(", ");
		uart_num2char(g_IrDA_Device[0].token[8]);
		uart_sendStr(", ");
		uart_num2char(g_IrDA_Device[0].token[9]);
		uart_sendStr(", ");
		UART_CR();
		UART_CR();
		delay(10);
	}
	return 0;
}
