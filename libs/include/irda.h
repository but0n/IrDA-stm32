#ifndef IRDA_H
#define IRDA_H

#define IRDA_NORMAL_STARTUP


#define IR1 TIM3->CCR1
#define IR2 TIM3->CCR2
#define IR3 TIM3->CCR3
#define IR4 TIM3->CCR4

void irda_PWM_Init(unsigned short arr, unsigned short psc);  //72MHz / (arr + 1)*(psc + 1)

#endif
