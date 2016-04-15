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
#define SAMPLERATE 20
volatile int16_t callibration = 0;
volatile bool flag_freq=0;
int16_t mpu[6];
// mpu[1] acc x
// mpu[2] acc y
//mpu [3] gyro x
//mpu[]  gyro y

float PID(float k, float i, float d,float e, float eprev,float integralprev){
	float integral=integralprev + (e+eprev)/2000*SAMPLERATE;
	float derevative=(e-eprev)/1000*SAMPLERATE;
	return k*e+i*integral+d*derevative;
}
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

		GPIO_InitTypeDef gpio;
		GPIO_StructInit(&gpio);
			gpio.GPIO_Pin = GPIO_Pin_5;
			gpio.GPIO_Mode = GPIO_Mode_Out_PP;
			GPIO_Init(GPIOA, &gpio);
float AccX;
float AccY;
float GyroX;
float GyroXprev=0;
float GyroXbias=0;
float GyroXsum=0;
float pAcc=1;//something different than 0
float pGyro;
float pFil;
float pFilprev=0;
float PIDout;
printf("Calibration stage 1...\r\n");
while(pAcc>0.1||pAcc<-0.1){
	MPU6050_GetRawAccelGyro(mpu);
AccX=mpu[1]*2.0f/32678.0f;
AccY=mpu[2]*2.0f/32678.0f;
pAcc=atan(AccX/AccY)*180/3.14;
}
pGyro=0;
printf("Calibration stage 2...\r\n");
delay_ms(1000);
for(int i=1;i<=10;i++){
	MPU6050_GetRawAccelGyro(mpu);
	GyroXsum+=mpu[3]*250.0f/32678.0f;
	delay_ms(50);
}
GyroXbias=GyroXsum/10;
MPU6050_GetRawAccelGyro(mpu);
pAcc=atan(AccX/AccY)*180/3.14;
pFil=pAcc;
GPIO_SetBits(GPIOA, GPIO_Pin_5);
printf("While started\r\n");
	while (1){
		if (flag_freq==1){
	//	forward(1);

		MPU6050_GetRawAccelGyro(mpu);
AccX=mpu[1]*2.0f/32678.0f;
AccY=mpu[2]*2.0f/32678.0f;
GyroX=mpu[3]*250.0f/32678.0f-GyroXbias;
pAcc=atan(AccX/AccY)*180/3.14; //in degres
pGyro=pGyro+(GyroX+GyroXprev)/2000*SAMPLERATE;
pFil=0.98*(pFil+(GyroX+GyroXprev)/2000*SAMPLERATE)+0.02*pAcc;//complementray filter
GyroXprev=GyroX;
PIDout=PID(1,0,5,pFil,pFilprev,0);
pFilprev=pFil;
//printf("%f\r\n",pAcc);
//printf("%f\r\n",pGyro);
printf("%.2f %.2f \r\n",pFil,PIDout);
if(PIDout>0)
	forward(PIDout);
else
	backward(-1*PIDout);
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
