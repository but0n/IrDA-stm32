#ifndef IRDA_H
#define IRDA_H
#include "bit.h"
#include "stm32f10x.h"
#include "uart.h"

//	NEC标准规定：红外通信的载波频率为38KHz，占空比为1:3
//	PWM 寄存器配置
//	计算细节: 72M/(21*90) 约等于 38k.
#define IR_PWM_ARR		90			// 计数器最大值
#define IR_PWM_PSC		21			// 计数器分频, 控制计数器频率
#define IR_PWM_CCR_DEF	30			// 占空比 1:3

#define WAVE_SEGEMENT_LENGTH	10000	//该值会影响学码函数的耗时, 因为最后一个电平的计时器会一直累加到该值函数才算超时
// 红外学码外设对象定义
#define IR_DEVICES_NUM			8		//共有8路学码发码外设
#define WAVE_SEGMEENT_NUM		400		//红外波形共有200次电平翻转

typedef struct {
	unsigned short token[WAVE_SEGMEENT_NUM];//保存红外波形的数组
	volatile unsigned long *IrInterrup;		//指针指向中断使能寄存器的某一位 (位带操作), 用来打开和关闭当前中断
	volatile unsigned long *IrPWM;			//指针指向 PWM 使能寄存器的某一位 (位带操作), 用来打开和关闭 PWM
	volatile unsigned long *signal;			//指针指向 GPIO ODR 的某一位, 用来检测输入红外波形的高低
} ir_st, *ir_pst;
ir_st g_IrDA_Device[IR_DEVICES_NUM];

//检测当前操作地址是否为数组最后一个元素
#define IR_ISOVERFLOW(curAddr, endAddr)	((curAddr) == ((endAddr)+WAVE_SEGMEENT_NUM-1))

//程序设置
#define IR_WAVE_FB_NUM	//在终端回馈红外波形数据
#define IR_AUTOENABLE	//在每次学码之后自动再次打开学码功能

#ifdef IR_WAVE_FB_NUM
	#define IR_WAVE_FEEDBACK(i) do {\
		uart_short2char(g_IrDA_Device[(i)].token[0]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[1]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[2]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[3]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[4]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[5]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[6]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[7]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[8]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[9]);\
		uart_sendStr(", ");\
\
		uart_short2char(g_IrDA_Device[(i)].token[10]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[11]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[12]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[13]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[14]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[15]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[16]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[17]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[18]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[19]);\
		uart_sendStr(", ");\
\
		uart_short2char(g_IrDA_Device[(i)].token[20]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[21]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[22]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[23]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[24]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[25]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[26]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[27]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[28]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[29]);\
		uart_sendStr(", ");\
\
		uart_short2char(g_IrDA_Device[(i)].token[30]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[31]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[32]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[33]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[34]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[35]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[36]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[37]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[38]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[39]);\
		uart_sendStr(", ");\
\
		uart_short2char(g_IrDA_Device[(i)].token[40]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[41]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[42]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[43]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[44]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[45]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[46]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[47]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[48]);\
		uart_sendStr(", ");\
		uart_short2char(g_IrDA_Device[(i)].token[49]);\
		uart_sendStr(", ");\
	} while(0)
#else
	#define IR_WAVE_FEEDBACK(i) do {} while (0)
#endif

#ifdef IR_AUTOENABLE
	#define IRQ_HANDLE_CORE(i) do {\
		*g_IrDA_Device[(i)].IrInterrup = 0;\
		irda_decode(&g_IrDA_Device[(i)]);\
		\
		IR_WAVE_FEEDBACK((i));\
		\
		UART_CR();\
		\
		*g_IrDA_Device[(i)].IrInterrup = 1;\
	} while(0)
#else
	#define IRQ_HANDLE_CORE(i) do {\
		*g_IrDA_Device[(i)].IrInterrup = 0;\
		irda_decode(&g_IrDA_Device[(i)]);\
		\
		IR_WAVE_FEEDBACK((i));\
		\
		UART_CR();\
	} while(0)
#endif


void irda_init();
void irda_PWM_Init();  //72MHz / (arr + 1)*(psc + 1)
void irda_EXTI_Init();
extern void delay_us(unsigned int t);
void irda_decode(ir_pst ir);
void irda_encode(ir_pst ir);

#endif
