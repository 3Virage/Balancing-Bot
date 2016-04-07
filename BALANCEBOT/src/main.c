/**
 ******************************************************************************
 * @file    main.c
 * @author  Ac6
 * @version V1.0
 * @date    01-December-2013
 * @brief   Default main function.
 ******************************************************************************
 */
#include <stdio.h>
#include "stm32f10x.h"
#include "uart.h"
#include "motor.h"
#include "MPU6050.h"
#include "HAL_MPU6050.h"
#include <math.h>
#define SAMPLERATE 500
volatile int16_t callibration = 0;
volatile bool flag_freq=0;
int16_t mpu[6];
// mpu[1] acc x
// mpu[2] acc y
//mpu [3] gyro x
//mpu[]  gyro y
int main(void) {
	RCC_APB2PeriphClockCmd(
				RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
						| RCC_APB2Periph_GPIOD, ENABLE);

	motor_init();
	uart_init();
	printf("Uart started\r\n");
	delay_init();
	MPU6050_I2C_Init();
	MPU6050_Initialize();
	printf("MPU started\r\n");
	//TIMER CONFIG
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

		TIM_TimeBaseInitTypeDef tim;
		NVIC_InitTypeDef nvic;

		TIM_TimeBaseStructInit(&tim);
		tim.TIM_CounterMode = TIM_CounterMode_Up;
		tim.TIM_Prescaler = 64000 - 1;
		tim.TIM_Period = SAMPLERATE - 1;
		TIM_TimeBaseInit(TIM3, &tim);

		TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

		TIM_Cmd(TIM3, ENABLE);



		nvic.NVIC_IRQChannel = TIM3_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 0;
		nvic.NVIC_IRQChannelSubPriority = 0;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
		printf("While started\r\n");
float AccX;
float AccY;
float GyroX;
float pAcc=0;
int i=0;
	while (1){
		if (flag_freq==1){
		forward(1);
		MPU6050_GetRawAccelGyro(mpu);
AccX=mpu[1]*2.0f/32678.0f;
AccY=mpu[2]*2.0f/32678.0f;
GyroX=mpu[3]*250.0f/32678.0f;
pAcc=atan(AccX/AccY)*180/3.14; //in degres

	printf("AccX:%.2f AccY:%.2f GyroX:%.2f PosAcc:%.2f\r\n",AccX,AccY,GyroX,pAcc);
flag_freq=0;
		}
	}
}

void TIM2_IRQHandler() {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		flag_motor = 1;
	}

}
void TIM3_IRQHandler() {
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
flag_freq=1;
	}

}
