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
	NVIC_EnableIRQ(EXTI0_IRQn);			//使能外部中断
	NVIC_SetPriority(EXTI0_IRQn, 0b0011);//设置中断优先级

	// 红外接收管的数据输出分别连接在单片机GPIO端口 C 的0, 1, 2, 3, 6, 7, 8, 9 引脚

	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;	//使能 AFIO 时钟, 中断属于复用功能
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;	//使能 IO Port C 时钟
//					76543210
	GPIOC->CRL &= 0x00FF0000;	//清空 0, 1, 2, 3, 6, 7
	GPIOC->CRL |= 0x88008888;	//配置为输入模式

	GPIOC->CRH &= 0xFFFFFF00;	//清空 8, 9
	GPIOC->CRH |= 0x00000088;	//配置为输入模式

	GPIOC->ODR |= 1 | 1<<1 | 1<<2 | 1<<3 | 1<<6 | 1<<7;			//上拉电阻

	AFIO->EXTICR[0] = 0x2222;	//使能C端口 0, 1, 2, 3 引脚的中断复用
	AFIO->EXTICR[1] = 0x2200;	//使能C端口 6, 7 引脚的中断复用
	AFIO->EXTICR[2] = 0x0022;	//使能C端口 8, 9 引脚的中断复用

	EXTI->FTSR |= 1 | 1<<1 | 1<<2 | 1<<3 | 1<<6 | 1<<7;			//下降沿触发
	EXTI->RTSR |= 1 | 1<<1 | 1<<2 | 1<<3 | 1<<6 | 1<<7;			//上升沿触发
}

void EXTI0_IRQHandler(void) {
	*g_IrDA_Device[0].IrInterrup = 0;	//屏蔽该中断, 保证学码不被打断
	irda_decode(&g_IrDA_Device[0]);		//decode	复制红外波形

	IR_WAVE_FEEDBACK(0);				//显示学习到的波形数据

	UART_CR();
#ifdef IR_AUTOENABLE					//如果定义了中断的自动使能
	*g_IrDA_Device[0].IrInterrup = 1;	//学码之后再次开启学码功能
#endif

	EXTI->PR |= 1<<0;					//在 PR 寄存器向当前中断位写 1 , 清除触发请求
}

void irda_init() {		// 串口外设初始化函数
	// irda_PWM_Init();	// 发送功能初始化
	irda_EXTI_Init();	// 接收功能初始化

	//实例化红外外设对象 - 第 1 路
	g_IrDA_Device[0].IrInterrup	= BIT_ADDRP(&(EXTI->IMR), 0);
	g_IrDA_Device[0].IrPWM		= BIT_ADDRP(&(TIM3->CCER), 0);
	g_IrDA_Device[0].signal		= BIT_ADDRP(&(GPIOC->IDR), 0);
}

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

void irda_encode(ir_pst ir) {
	unsigned short *wave = ir->token;	//定义一个指针指向被发送的波形数据
	unsigned short cnt;					//计数器
	while((cnt = *wave++)) {	//提取当前波形长度, 如果长度值有效则将发送波形, 并且将 wave 指向下一段数据
		*ir->IrPWM ^= 1;		//电平翻转,切换 PWM 输出状态,如果之前输出是打开的则关闭, 如果之前是关闭的则打开
		while(cnt--)
			delay_us(20);		//计数器单位为 20us, 因为波形有38KHz载波
	}
}
