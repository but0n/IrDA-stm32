#include "stm32f10x.h"
#include "uart.h"

int main() {
	uart_init(72, 115200);

	while(1);
	return 0;
}
