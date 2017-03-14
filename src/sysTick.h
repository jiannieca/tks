/***********************************************************************
文件名称：SysTick.h
功    能：
编写时间：2013.4.25
编 写 人：
注    意：
***********************************************************************/

#ifndef _SYSTICK_H_
#define _SYSTICK_H_

void SysTick_Configuration(void);
void Delay(__IO uint32_t nTime);
void TimingDelay_Decrement(void);
uint8_t isDelayExp( uint32_t delayTime, uint32_t startTime );

extern __IO uint32_t LocalTime ;
#define TickGetDiff(a,b) ((b <= a ? a - b : ((uint32_t)0xFFFFFFFF - b) + a))

#endif 
