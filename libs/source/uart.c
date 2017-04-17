#include "uart.h"
#include "stm32f10x.h"
#include "irda.h"
int top = -1;	//Stack Pointer
char gCmdCache[CMD_MAX_LENGTH];


void uart_init(unsigned int pclk2, unsigned int bound) {
    float temp;
    unsigned short mantissa;
    unsigned short fraction;
    temp = (float)(pclk2*1000000)/(bound*16);
    mantissa = temp;
    fraction = (temp - mantissa) * 16;
    mantissa <<= 4;
    mantissa += fraction;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   //GPIOA Enable
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  //USART1 Enable

    GPIOA->CRH &= 0xFFFFF00F;
    GPIOA->CRH |= 0x000008B0;

    RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;

    USART1->BRR = mantissa;
    USART1->CR1 |= 0x200C;	//Set up UE, TE and RE

    USART1->CR1 |= 1<<8;	//PE Interrup Enable
    USART1->CR1 |= 1<<5;	//RX Not Empty Interrup Enable


    USART1->SR;     //Read Reg SR to Clean TXE and TE,(Reset value: 0x00C0)

	// SCB->AIRCR &= 0x05FAF8FF;	// AIRCE Key: 0x05FA
	// SCB->AIRCR |= 0x05FA0400;	// Set up group value
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn, 0b1001);

}

void USART1_IRQHandler(void) {
	if(USART1->SR & USART_SR_RXNE) {
		char cmd = USART1->DR;	// Read this register to clear RXNE flag
		switch (cmd) {
			case 0x0D:
			case 0x0A:
				UART_CR();
				uart_sendStr("Handle Command:\t");
				uart_sendStr(gCmdCache);
				UART_CR();
				clrCache();
				break;
			case 0x08:
			case 0x7F:
				pop = '\0';
				uart_sendData(0x7F);
				uart_sendData(0x08);
				break;
			case '$':
				clrCache();
			default:
				if(STACK_OVERFLOW)
					break;
				push(cmd);
				uart_sendData(cmd);
				break;
		}
	}
}

void uart_decode() {
//	The Last Item Of CMD Cache Array
//	| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |-bit-|
//	Each bit is a switch of IR Devices
	char k;
	// unsigned char stagement;
	while(top > -1) {	// While CMD Cache is Not Empty
		k = pop;
		if(ISLEGAL_NUM(k)) {	// Handle Numbers
			gCmdCache[CMD_MAX_LENGTH - 1] |= 1 << (k - '0');	// Store number argument to the top of cmd stack
		} else {	// Handle Key Token
			if(k == TOKEN_SEND) {	// Send data

			}
			else if(k == TOKEN_LEARN) {	// decode and store data

			}
		}




	}
}

void uart_sendData(unsigned char data) {
    USART1->DR = data;
    while((USART1->SR & 0x40) == 0);
}

void uart_sendStr(char *cmd) {
	while(*cmd)
		uart_sendData(*cmd++);
}

void uart_num2char(unsigned int k) {
	char cache[] = "0000000000";	// Max value is 4294967295
	unsigned char i = 9;
	unsigned int bit[] = {1000000000, 100000000, 10000000, 1000000, 100000, 10000, 1000, 100, 10, 1};

	do {
		cache[i] += (char)(k / bit[i] % 10);
	} while(i--);
	uart_sendStr(cache);
}
