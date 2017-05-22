#include "irda.h"

ir_st g_IrDA_Device[IR_DEVICES_NUM];

//红外学码电路发码部分初始化函数
void irda_PWM_Init() {
	// 红外发射管分别依次连接在单片机的 TIM3_CH1, TIM3_CH2, TIM3_CH3, TIM3_CH4,
	//		TIM4_CH1, TIM4_CH2, TIM4_CH3, TIM4_CH4
	//	所以在这个函数中需要初始化 TIM3 和 TIM4

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;	//TIM3 Enable
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;	//IO Port A and B Enable

	// GPIOA GPIOB 寄存器配置
    GPIOA->CRL &= 0x00FFFFFF;
    GPIOA->CRL |= 0xBB000000;   //将 6, 7 脚配置为复用推挽输出
    GPIOA->ODR |= 1<<7;			//TIM3_CH2 GPIO 配置
    GPIOA->ODR |= 1<<6;			//TIM3_CH1 GPIO 配置

    GPIOB->CRL &= 0x00FFFF00;
    GPIOB->CRL |= 0xBB0000BB;	//将 0, 1, 6, 7 脚配置为复用推挽输出

	GPIOB->CRH &= 0xFFFFFF00;
	GPIOB->CRH |= 0x000000BB;	//将 8, 9 脚配置为复用推挽输出

	GPIOB->ODR |= 1<<0 | 1<<1 | 1<<6 | 1<<7 | 1<<8 | 1<<9;

	// TIM3 寄存器配置
    TIM3->ARR = IR_PWM_ARR - 1;	//配置计数器最大值
    TIM3->PSC = IR_PWM_PSC - 1;	//配置计数器分频

    TIM3->CCMR1 |= 6<<4;			//CH1 设置为 OC1M[2:0]: PWM 模式
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;	//CH1 设置为 OC1PE: 使能
    TIM3->CCMR1 |= 6<<12;   		//CH2 设置为 OC2M[2:0]: PWM 模式
    TIM3->CCMR1 |= TIM_CCMR1_OC2PE;	//CH2 设置为 OC2PE: 使能
    TIM3->CCMR2 |= 6<<4;    		//CH3 设置为 OC3M[2:0]: PWM 模式
    TIM3->CCMR2 |= TIM_CCMR2_OC3PE;	//CH3
    TIM3->CCMR2 |= 6<<12;			//CH4
    TIM3->CCMR2 |= TIM_CCMR2_OC4PE;	//CH4

	TIM3->CCR1 = TIM3->CCR2 = TIM3->CCR3 = TIM3->CCR4 = IR_PWM_CCR_DEF;
	//CH1 ~ CH4 计数器使能
    // TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    TIM3->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;	//配置APRE位并使能计数器

	// TIM4 寄存器配置
    TIM4->ARR = IR_PWM_ARR - 1;	//配置计数器最大值
    TIM4->PSC = IR_PWM_PSC - 1;	//配置计数器分频

    TIM4->CCMR1 |= 6<<4;			//CH1 设置为 OC1M[2:0]: PWM 模式
    TIM4->CCMR1 |= TIM_CCMR1_OC1PE;	//CH1 设置为 OC1PE: 使能
    TIM4->CCMR1 |= 6<<12;   		//CH2 设置为 OC2M[2:0]: PWM 模式
    TIM4->CCMR1 |= TIM_CCMR1_OC2PE;	//CH2 设置为 OC2PE: 使能
    TIM4->CCMR2 |= 6<<4;    		//CH3 设置为 OC3M[2:0]: PWM 模式
    TIM4->CCMR2 |= TIM_CCMR2_OC3PE;	//CH3
    TIM4->CCMR2 |= 6<<12;			//CH4
    TIM4->CCMR2 |= TIM_CCMR2_OC4PE;	//CH4

	TIM4->CCR1 = TIM4->CCR2 = TIM4->CCR3 = TIM4->CCR4 = IR_PWM_CCR_DEF;
	//CH1 ~ CH4 计数器使能
    // TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    TIM4->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;	//配置APRE位并使能计数器





    //TIM1->BDTR |= 1<<15;  //高级定时器需要使能BDTR寄存器
}

//红外学码电路收码部分初始化函数
void irda_EXTI_Init() {
	//使能外部中断
	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI2_IRQn);
	NVIC_EnableIRQ(EXTI3_IRQn);
	NVIC_EnableIRQ(EXTI9_5_IRQn);

	NVIC_SetPriority(EXTI0_IRQn, 0b0011);	//设置中断优先级
	NVIC_SetPriority(EXTI1_IRQn, 0b0011);	//设置中断优先级
	NVIC_SetPriority(EXTI2_IRQn, 0b0011);	//设置中断优先级
	NVIC_SetPriority(EXTI3_IRQn, 0b0011);	//设置中断优先级
	NVIC_SetPriority(EXTI9_5_IRQn, 0b0011);	//设置中断优先级



	// 红外接收管的数据输出分别连接在单片机GPIO端口 C 的0, 1, 2, 3, 6, 7, 8, 9 引脚

	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;	//使能 AFIO 时钟, 因为中断属于复用功能
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;	//使能 IO Port C 时钟
//					76543210
	GPIOC->CRL &= 0x00FF0000;	//清空 0, 1, 2, 3, 6, 7
	GPIOC->CRL |= 0x88008888;	//配置为输入模式

	GPIOC->CRH &= 0xFFFFFF00;	//清空 8, 9
	GPIOC->CRH |= 0x00000088;	//配置为输入模式

	GPIOC->ODR |= 1 | 1<<1 | 1<<2 | 1<<3 | 1<<6 | 1<<7 | 1<<8 | 1<<9;	//上拉电阻

	AFIO->EXTICR[0] = 0x2222;	//使能C端口 0, 1, 2, 3 引脚的中断复用
	AFIO->EXTICR[1] = 0x2200;	//使能C端口 6, 7 引脚的中断复用
	AFIO->EXTICR[2] = 0x0022;	//使能C端口 8, 9 引脚的中断复用

	EXTI->FTSR |= 1 | 1<<1 | 1<<2 | 1<<3 | 1<<6 | 1<<7 | 1<<8 | 1<<9;	//下降沿触发
	EXTI->RTSR |= 1 | 1<<1 | 1<<2 | 1<<3 | 1<<6 | 1<<7 | 1<<8 | 1<<9;	//上升沿触发
}

//各路的中断处理函数
void EXTI0_IRQHandler(void) {
	IRQ_HANDLE_CORE(0);
	EXTI->PR |= 1<<0;
}

void EXTI1_IRQHandler(void) {
	IRQ_HANDLE_CORE(1);
	EXTI->PR |= 1<<1;
}

void EXTI2_IRQHandler(void) {
	IRQ_HANDLE_CORE(2);
	EXTI->PR |= 1<<2;
}

void EXTI3_IRQHandler(void) {
	IRQ_HANDLE_CORE(3);
	EXTI->PR |= 1<<3;
}

void EXTI9_5_IRQHandler(void) {
	unsigned long origin = EXTI->PR;

	if(origin & EXTI_PR_PR6) {			//如果来源为 6
		IRQ_HANDLE_CORE(4);
		EXTI->PR |= 1<<6;
	} else if(origin & EXTI_PR_PR7) {	//如果来源为 7
		IRQ_HANDLE_CORE(5);
		EXTI->PR |= 1<<7;
	} else if(origin & EXTI_PR_PR8) {	//如果来源为 8
		IRQ_HANDLE_CORE(6);
		EXTI->PR |= 1<<8;
	} else if(origin & EXTI_PR_PR9) {	//如果来源为 9
		IRQ_HANDLE_CORE(7);
		EXTI->PR |= 1<<9;
	}
}


//红外学码电路初始化函数
void irda_init() {
	irda_PWM_Init();	// 发送功能初始化
	irda_EXTI_Init();	// 接收功能初始化

	//实例化红外外设对象 - 第 1 路
	g_IrDA_Device[0].IrInterrup	= BIT_ADDRP(&(EXTI->IMR), 0);
	g_IrDA_Device[0].IrPWM		= BIT_ADDRP(&(TIM3->CCER), TIM_CCER_CC1E);
	g_IrDA_Device[0].signal		= BIT_ADDRP(&(GPIOC->IDR), 0);

	//实例化红外外设对象 - 第 2 路
	g_IrDA_Device[1].IrInterrup	= BIT_ADDRP(&(EXTI->IMR), 1);
	g_IrDA_Device[1].IrPWM		= BIT_ADDRP(&(TIM3->CCER), TIM_CCER_CC2E);
	g_IrDA_Device[1].signal		= BIT_ADDRP(&(GPIOC->IDR), 1);

	//实例化红外外设对象 - 第 3 路
	g_IrDA_Device[2].IrInterrup	= BIT_ADDRP(&(EXTI->IMR), 2);
	g_IrDA_Device[2].IrPWM		= BIT_ADDRP(&(TIM3->CCER), TIM_CCER_CC3E);
	g_IrDA_Device[2].signal		= BIT_ADDRP(&(GPIOC->IDR), 2);

	//实例化红外外设对象 - 第 4 路
	g_IrDA_Device[3].IrInterrup	= BIT_ADDRP(&(EXTI->IMR), 3);
	g_IrDA_Device[3].IrPWM		= BIT_ADDRP(&(TIM3->CCER), TIM_CCER_CC4E);
	g_IrDA_Device[3].signal		= BIT_ADDRP(&(GPIOC->IDR), 3);

	//实例化红外外设对象 - 第 5 路
	g_IrDA_Device[4].IrInterrup	= BIT_ADDRP(&(EXTI->IMR), 6);
	g_IrDA_Device[4].IrPWM		= BIT_ADDRP(&(TIM4->CCER), TIM_CCER_CC1E);
	g_IrDA_Device[4].signal		= BIT_ADDRP(&(GPIOC->IDR), 6);

	//实例化红外外设对象 - 第 6 路
	g_IrDA_Device[5].IrInterrup	= BIT_ADDRP(&(EXTI->IMR), 7);
	g_IrDA_Device[5].IrPWM		= BIT_ADDRP(&(TIM4->CCER), TIM_CCER_CC2E);
	g_IrDA_Device[5].signal		= BIT_ADDRP(&(GPIOC->IDR), 7);

	//实例化红外外设对象 - 第 7 路
	g_IrDA_Device[6].IrInterrup	= BIT_ADDRP(&(EXTI->IMR), 8);
	g_IrDA_Device[6].IrPWM		= BIT_ADDRP(&(TIM4->CCER), TIM_CCER_CC3E);
	g_IrDA_Device[6].signal		= BIT_ADDRP(&(GPIOC->IDR), 8);

	//实例化红外外设对象 - 第 8 路
	g_IrDA_Device[7].IrInterrup	= BIT_ADDRP(&(EXTI->IMR), 9);
	g_IrDA_Device[7].IrPWM		= BIT_ADDRP(&(TIM4->CCER), TIM_CCER_CC4E);
	g_IrDA_Device[7].signal		= BIT_ADDRP(&(GPIOC->IDR), 9);

}

//红外信号学码函数
void irda_decode(ir_pst ir) {
	unsigned char lastStatus = *ir->signal;		//用来保存上一次电平状态, 以判断电平是否发生翻转
	unsigned short *wave = ir->token;			//指向用来存储波形的数组

	for(unsigned int cnt = 0; cnt < WAVE_SEGEMENT_LENGTH; cnt++) {	//计数器递增, 计数器的数值代表波形长度
		if(lastStatus != *ir->signal) {
			*wave++ = cnt;						//当发生电平跳转时保存当前计数并将指针指向波形数组的下一个元素
			if(IR_ISOVERFLOW(wave, ir->token)) {//如果遥控器按键未松开会导致多次发送红外信号
				uart_sendStr("\n\r波形数据溢出, 请松开按键!\n\r");
				return;
			}
			cnt = 0;							//清空计数器以便于测量下一段波形长度
		}
		lastStatus = *ir->signal;
		delay_us(20);	// 跳过38KHz载波信号的电平反转
	}
	unsigned short len = wave - ir->token + 1;	//计算波形的高电平和低电平共有多少段
	uart_sendStr("\n\r波形数组长度:\t");
	uart_short2char(len);						//在终端反馈数据
	UART_CR();

}

//红外信号发码函数
void irda_encode(ir_pst ir) {
	unsigned short *wave = ir->token;	//定义一个指针指向被发送的波形数据
	unsigned short cnt;					//计数器
	while((cnt = *wave++)) {	//提取当前波形长度, 如果长度值有效则将发送波形, 并且将 wave 指向下一段数据
		*ir->IrPWM ^= 1;		//电平翻转,切换 PWM 输出状态,如果之前输出是打开的则关闭, 如果之前是关闭的则打开
		while(cnt--)
			delay_us(20);		//计数器单位为 20us, 因为波形有38KHz载波
	}
}
