#ifndef IRDA_H
#define IRDA_H
#include "bit.h"
#include "stm32f10x.h"
#include "uart.h"
//	NEC标准规定：红外通信的载波频率为38KHz，占空比为1:3

#define IR_PWM_ARR		90			// 计数器最大值
#define IR_PWM_PSC		21			// 计数器分频, 控制计数器频率
#define IR_PWM_CCR_DEF	30			// 占空比 1:3

#define IR1 BIT_ADDR(TIM3->CCER, TIM_CCER_CC1E)
#define IR2 BIT_ADDR(TIM3->CCER, TIM_CCER_CC2E)
#define IR3 BIT_ADDR(TIM3->CCER, TIM_CCER_CC3E)
#define IR4 BIT_ADDR(TIM3->CCER, TIM_CCER_CC4E)

#define WAVE_SEGEMENT_LENGTH	10000	//该值会影响学码函数的耗时, 因为最后一个电平的计时器会一直累加到该值函数才会结束
// 红外学码外设对象定义
#define IR_DEVICES_NUM	8	//共有8路学码发码外设
#define WAVE_SEGMEENT_NUM		400	//红外波形共有200次电平翻转
typedef struct {
	unsigned short token[WAVE_SEGMEENT_NUM];
	volatile unsigned long *IrInterrup;	//this reg will be a Pointer to EXTI->IMR, for enable or disable
	volatile unsigned long *IrPWM;
	volatile unsigned long *signal;
} ir_st, *ir_pst;
ir_st g_IrDA_Device[IR_DEVICES_NUM];
//检测当前操作地址是否为数组最后一个元素
#define IR_ISOVERFLOW(curAddr, endAddr)	((curAddr) == ((endAddr)+WAVE_SEGMEENT_NUM-1))

// Address
#define INT_ENABLE_ADDR		EXTI_BASE			//IMR
#define PWM_ENABLE_ADDR		TIM3_BASE+0x20		//TIM3_CCER
#define ID_REG_ADDR			GPIOA_BASE+0x08		//GPIOA_IDR


void irda_init();
void irda_PWM_Init();  //72MHz / (arr + 1)*(psc + 1)
void irda_EXTI_Init();
extern void delay_us(unsigned int t);
void irda_decode(ir_pst ir);

#endif
