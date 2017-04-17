#include "irda.h"

ir_st g_IrDA_Device[IR_DEVICES_NUM];

//A7
void irda_PWM_Init() {

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;		//TIM3 Enable
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;		//IO Port A Enable
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;		//IO Port B for TIM3 Channel 3 and 4

    GPIOA->CRL &= 0x00FFFFFF;   //Clean
    GPIOA->CRL |= 0xBB000000;   //复用推挽输出
    GPIOA->ODR |= 1<<7;			//CH2 GPIO Config
    GPIOA->ODR |= 1<<6;			//CH1 GPIO Config

    GPIOB->CRL &= 0xFFFFFF00;
    GPIOB->CRL |= 0x000000BB;
    GPIOB->ODR |= 1;
    GPIOB->ODR |= 1<<1;

    TIM3->ARR = IR_PWM_ARR - 1;
    TIM3->PSC = IR_PWM_PSC - 1;

    TIM3->CCMR1 |= 6<<4;    //CH1 Set OC1M[2:0]: PWM Mode
    TIM3->CCMR1 |= 1<<3;    //CH1 Set OC1PE: Enable
    TIM3->CCMR1 |= 6<<12;   //CH2 Set OC2M[2:0]: PWM Mode
    TIM3->CCMR1 |= 1<<11;   //CH2 Set OC2PE: Enable
    TIM3->CCMR2 |= 6<<4;    //CH3 Set OC3M[2:0]: PWM Mode
    TIM3->CCMR2 |= 1<<3;    //CH3
    TIM3->CCMR2 |= 6<<12;   //CH4
    TIM3->CCMR2 |= 1<<11;

    TIM3->CCER |= 1;        //CH1 Output Enable
    TIM3->CCER |= 1<<4;     //CH2 Output Enable
    TIM3->CCER |= 1<<8;
    TIM3->CCER |= 1<<12;

    TIM3->CR1 = 0x80;       //APRE Enable
    TIM3->CR1 |= 1;         //Set CEN, Allow to Count
    //TIM1->BDTR |= 1<<15;  //高级定时器需要使能BDTR寄存器
}

void irda_EXTI_Init() {
	NVIC_EnableIRQ(EXTI2_IRQn);
	NVIC_SetPriority(EXTI2_IRQn, 0b0011);

	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

	GPIOA->CRL &= 0xFFFFF0FF;	//清空
	GPIOA->CRL |= 0x00000800;	//配置为输入模式
	GPIOA->ODR |= 1<<2;			//上拉电阻

	AFIO->EXTICR[0] |= 0x0000;
	EXTI->FTSR |= 1<<2;			//下降沿触发
	EXTI->RTSR |= 1<<2;			//上升沿触发
}

void EXTI2_IRQHandler(void) {
	// EXTI->IMR &= ~(1<<2);	//屏蔽该中断
	*(g_IrDA_Device[0].IrInterrup) = 0;
	irda_decode(&g_IrDA_Device[0]);
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
	uart_num2char(g_IrDA_Device[0].token[10]);
	uart_sendStr(", ");
	uart_num2char(g_IrDA_Device[0].token[11]);
	uart_sendStr(", ");
	uart_num2char(g_IrDA_Device[0].token[12]);
	uart_sendStr(", ");
	uart_num2char(g_IrDA_Device[0].token[13]);
	uart_sendStr(", ");
	uart_num2char(g_IrDA_Device[0].token[14]);
	uart_sendStr(", ");
	uart_num2char(g_IrDA_Device[0].token[15]);
	uart_sendStr(", ");
	uart_num2char(g_IrDA_Device[0].token[16]);
	uart_sendStr(", ");
	UART_CR();

	EXTI->IMR |= 1<<2;	//开放该中断
	EXTI->PR |= 1<<2;	//向该位写 1 , 清除触发请求
}

void irda_init() {
	// irda_PWM_Init();	// 发送功能初始化
	irda_EXTI_Init();	// 接收功能初始化

	g_IrDA_Device[0].IrInterrup	= (volatile unsigned long *)BITBAND(INT_ENABLE_ADDR, 2);
	g_IrDA_Device[0].IrPWM		= (volatile unsigned long *)BITBAND(PWM_ENABLE_ADDR, 0);
	g_IrDA_Device[0].signal		= (volatile unsigned long *)BITBAND(ID_REG_ADDR, 2);
	*(g_IrDA_Device[0].IrInterrup) = 1;		//开放该外设的中断请求

}

void irda_decode(ir_pst ir) {
	unsigned char lastStatus = *ir->signal;
	unsigned int *wave = ir->token;
	for(unsigned int cnt = 0; cnt < WAVE_SEGEMENT_LENGTH; cnt++) {
		if(lastStatus != *ir->signal) {
			//当发生电平跳转时保存当前计数并清空计数器以便于记录下次数据
			*wave++ = cnt;
			cnt = 0;
		}
		lastStatus = *ir->signal;
		cnt++;

	}
	unsigned int len = wave - ir->token;
	UART_CR();
	uart_num2char(len);
	UART_CR();

}
