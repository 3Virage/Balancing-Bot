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
#include "MPU6050.c"
#include "HAL_MPU6050.h"
#define GYROSAMPLETIME 500
volatile int16_t callibration = 0;

volatile int16_t Gyro[6];
volatile float k =5;
volatile uint8_t dt = 3;
volatile float pos;
volatile int time = 0;
volatile int16_t vcur = 0;
volatile float vprevfil = 0;
volatile float vcurreal = 0;
volatile float vcurfil = 0;
volatile bool gyro_flag;

int main(void) {

	motor_init();
	uart_init();
	delay_init();
	MPU6050_I2C_Init();
	MPU6050_Initialize();

	TIM_TimeBaseInitTypeDef tim;
	NVIC_InitTypeDef nvic;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_TimeBaseStructInit(&tim);
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Prescaler = 6400 - 1;
	tim.TIM_Period = GYROSAMPLETIME - 1;
	TIM_TimeBaseInit(TIM3, &tim);

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	TIM_Cmd(TIM3, ENABLE);

	nvic.NVIC_IRQChannel = TIM3_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);*/

/*
	 float check=0;
	 MPU6050_GetRawAccelGyro(Gyro);
	 while(check<0.9 || check>0.93){
	 MPU6050_GetRawAccelGyro(Gyro);
	 check=Gyro[2]* 2.0f / 32678.0f;
	 pos=0;
	 }
*/
	/*
int i=0;
	int sum = 0;
	for(i=0;i<=10;i++) {
		MPU6050_GetRawAccelGyro(Gyro);
		sum+=Gyro[i];
		delay_ms(5);
	}*
	callibration = sum / 10;*/
	/*while (1) {

		l_forward(100);
		r_backward(100);
		if(gyro_flag){
			gyro_flag=0;
		MPU6050_GetRawAccelGyro(Gyro);
		time=timer_ms_get();
		timer_ms_rst();

					vprevfil = vcurfil;
					vcur = Gyro[3] - callibration;
					vcurreal = vcur * 250.0f / 32678.0f ;
					printf("Vel:%.2f Velf:%.2f Pos:%.2f  time:%d\r\n", vcurreal, vcurfil,
							pos, time);
					vcurfil = (vcurfil * dt + vcurreal) / (dt + 1);
					pos = pos + (vcurfil + vprevfil) * time / 2000;
					if(Gyro[2]>0.89 && Gyro[2]<0.93)pos=0;
				if (pos > 0) {
			backward(pos * k);
		} else {
			forward(pos * k * (-1));
		}
gyro_flag=0;
}*/
	}
}

void TIM2_IRQHandler() {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		flag_motor = 1;
	}

}
/*
void TIM3_IRQHandler() {
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	gyro_flag=1;

	}
}*/
