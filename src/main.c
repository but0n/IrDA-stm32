#include "stm32f10x.h"
#include "uart.h"
#define NVIC_GROUPING	3
int main() {
	NVIC_SetPriorityGrouping(0x07 - NVIC_GROUPING);
	// 设置中断优先级分组为3, 在寄存器中: xxx.y (x 指抢占式优先级, y 指响应式优先级)

	uart_init(72, 115200);
	while(1);
	return 0;
}
