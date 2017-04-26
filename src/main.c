#include "stm32f10x.h"
#include "uart.h"
#include "irda.h"

// 设置中断优先级分组为3, 在寄存器中: xxx.y (x 指抢占式优先级, y 指响应式优先级)
#define NVIC_GROUPING	3

// 延时函数, 单位为毫秒
void delay_ms(unsigned int t) {
	SysTick->LOAD = 9000 * t;
	SysTick->VAL = 0;
	SysTick->CTRL = 0x01;
	for(unsigned int tmp = SysTick->CTRL;(tmp&0x01)&&(!(tmp&SysTick_CTRL_COUNTFLAG));tmp = SysTick->CTRL);
	SysTick->CTRL = 0;
	SysTick->VAL = 0;
}

// 延时函数, 单位为微秒
void delay_us(unsigned int t) {
	SysTick->LOAD = 9 * t;
	SysTick->VAL = 0;
	SysTick->CTRL = 0x01;
	for(unsigned int tmp = SysTick->CTRL;(tmp&0x01)&&(!(tmp&SysTick_CTRL_COUNTFLAG));tmp = SysTick->CTRL);
	SysTick->CTRL = 0;
	SysTick->VAL = 0;
}

int main() {
	// 配置中断优先级分组
	NVIC_SetPriorityGrouping(0x07 - NVIC_GROUPING);

	uart_init(72, 115200);	// 初始化串口, 波特率为115200
	irda_init();			// irda初始化
	while(1) {
		// uart_sendStr("Alive~");
		// UART_CR();
		// delay_ms(4000);
	}
	return 0;
}
