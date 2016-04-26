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
#define SAMPLERATE 25
#define PIDMAX 2000


float integral=0;
int16_t mpu[6];
// mpu[1] acc x
// mpu[2] acc y
//mpu [3] gyro x


float PID(float k, float i, float d,float e, float eprev){
	integral+= e*SAMPLERATE/1000;
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
float AccX=0;
float AccY=0;

float GyroX=0;
float GyroXprev=0;
float GyroXbias=0;
float GyroXsum=0;

float pAcc=1;//something different than 0
float pFil=0;
float pWheel=0;

float PID1out=0;
float PID2out=0;
float error1=0;
float error1prev=0;
float error2=0;
float error2prev=0;
printf("Calibration stage 1...\r\n");
while(pAcc>0.01||pAcc<-0.01){
	MPU6050_GetRawAccelGyro(mpu);
AccX=mpu[1]*2.0f/32678.0f;
AccY=mpu[2]*2.0f/32678.0f;
pAcc=atan(AccX/AccY)*180/3.14;
}

printf("Calibration stage 2...\r\n");
GPIO_SetBits(GPIOA, GPIO_Pin_5);
delay_ms(500);
GPIO_ResetBits(GPIOA, GPIO_Pin_5);
for(int i=1;i<=10;i++){
	MPU6050_GetRawAccelGyro(mpu);
	GyroXsum+=mpu[3]*250.0f/32678.0f;
	delay_ms(10);
}
GPIO_SetBits(GPIOA, GPIO_Pin_5);
GyroXbias=GyroXsum/10;
printf("%.2f",GyroXbias);
MPU6050_GetRawAccelGyro(mpu);
pAcc=atan(AccX/AccY)*180/3.14;
pFil=0;
printf("While started\r\n");

	while (1){
		while (!flag_freq);
		flag_freq=0;
MPU6050_GetRawAccelGyro(mpu);
AccX=mpu[1]*2.0f/32678.0f;
AccY=mpu[2]*2.0f/32678.0f;
GyroX=mpu[3]*250.0f/32678.0f-GyroXbias;
pAcc=(pAcc*9+atan(AccX/AccY)*180/3.14)/10; //in degres

pFil=0.96*(pFil+((GyroX+GyroXprev)*SAMPLERATE/2000))+(0.04*pAcc);//complementray filter

GyroXprev=GyroX;

error1=0-pFil;
PID1out=PID(3,0,0.5,error1,error1prev);
error2=pWheel;
PID2out=PID(0,0,0,error2,error2prev);
error1prev=error1;
error2prev=error2;


//printf("%f\r\n",pAcc);
//printf("%f\r\n",pGyro);

PID2out=PID1out; //fix here
if(PID2out>PIDMAX)PID2out=PIDMAX;
else if(PID2out<-PIDMAX)PID2out=-PIDMAX;

//printf("%.2f %.2f %.2f \r\n",pFil,pAcc,PID2out);

pWheel+=PID2out;

if(PID2out>0)
	forward(PID2out);
else
	backward(-1*PID2out);

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
