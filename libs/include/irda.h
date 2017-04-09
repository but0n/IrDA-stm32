#ifndef IRDA_H
#define IRDA_H
#include "bit.h"
//	NEC标准规定：红外通信的载波频率为38KHz，占空比为1:3

#define IR_PWM_ARR		90			// 计数器最大值
#define IR_PWM_PSC		21			// 计数器分频, 控制计数器频率
#define IR_PWM_CCR_DEF	30			// 占空比 1:3

#define IR1 BIT_ADDR(TIM3->CCER, TIM_CCER_CC1E)
#define IR2 BIT_ADDR(TIM3->CCER, TIM_CCER_CC2E)
#define IR3 BIT_ADDR(TIM3->CCER, TIM_CCER_CC3E)
#define IR4 BIT_ADDR(TIM3->CCER, TIM_CCER_CC4E)

void irda_init();
void irda_PWM_Init();  //72MHz / (arr + 1)*(psc + 1)
void irda_EXTI_Init();
extern void delay(volatile unsigned int count);

#endif
