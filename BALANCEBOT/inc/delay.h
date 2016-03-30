/*
 * delay.c
 *
 *  Created on: 5 lut 2016
 *      Author: Karol
 */
#include "stm32f10x.h"
volatile uint32_t timer_ms=0;
void SysTick_Handler()
{
	timer_ms++;

}
void timer_ms_rst(void){
	timer_ms=0;
}
void delay_ms(int time)
{
	timer_ms=0;
	while(timer_ms<time);
}

void delay_init(){
	SysTick_Config(SystemCoreClock/1000);
}

uint32_t timer_ms_get(void){return  timer_ms;}
