#include "stm32f10x.h"
#include "delay.h"
#include "MPU6050.h" //zawiera definicje bool

#define LMOTOR_PIN1 GPIO_Pin_0
#define LMOTOR_PIN2 GPIO_Pin_1
#define LMOTOR_PIN3 GPIO_Pin_2
#define LMOTOR_PIN4 GPIO_Pin_3
#define RMOTOR_PIN1 GPIO_Pin_9
#define RMOTOR_PIN2 GPIO_Pin_10
#define RMOTOR_PIN3 GPIO_Pin_11
#define RMOTOR_PIN4 GPIO_Pin_12
#define MOTOR_GPIO GPIOC
#define MOTORDELAY 30 //time in 0.1 ms beetwen turning on motor coils, 14 is minimum

volatile uint8_t lkr = 0;
volatile uint8_t rkr = 0;
volatile uint8_t kr=0;
volatile bool flag_motor = 0;



void l_forward(int steps) {

	while (steps) {
		while (!flag_motor)
			;
		flag_motor = 0;
		switch (lkr) {
		case 0:
			GPIO_Write(MOTOR_GPIO, LMOTOR_PIN4);
			break;
		case 1:
			GPIO_Write(MOTOR_GPIO, LMOTOR_PIN2);
			break;
		case 2:
			GPIO_Write(MOTOR_GPIO, LMOTOR_PIN3);
			break;
		case 3:
			GPIO_Write(MOTOR_GPIO, LMOTOR_PIN1);
			break;
		default:
			lkr = 0;
			break;
		}
		if (++lkr > 3)
			lkr = 0;
		steps--;
	}
}

void l_backward(int steps) {
	while (steps) {
		while (!flag_motor)
			;
		flag_motor = 0;
		switch (lkr) {
		case 0:
			GPIO_Write(MOTOR_GPIO, LMOTOR_PIN3);
			break;
		case 1:
			GPIO_Write(MOTOR_GPIO, LMOTOR_PIN2);
			break;
		case 2:
			GPIO_Write(MOTOR_GPIO, LMOTOR_PIN4);
			break;
		case 3:
			GPIO_Write(MOTOR_GPIO, LMOTOR_PIN1);
			break;
		default:
			lkr = 0;
			break;
		}
		if (++lkr > 3)
			lkr = 0;
		steps--;
	}
}

void r_forward(int steps) {

	while (steps) {
		while (!flag_motor)
			;
		flag_motor = 0;
		switch (rkr) {
		case 0:
			GPIO_Write(MOTOR_GPIO, RMOTOR_PIN4);
			break;
		case 1:
			GPIO_Write(MOTOR_GPIO, RMOTOR_PIN2);
			break;
		case 2:
			GPIO_Write(MOTOR_GPIO, RMOTOR_PIN3);
			break;
		case 3:
			GPIO_Write(MOTOR_GPIO, RMOTOR_PIN1);
			break;
		default:
			rkr = 0;
			break;
		}
		if (++rkr > 3)
			rkr = 0;
		steps--;
	}
}

void r_backward(int steps) {
	while (steps) {
		while (!flag_motor)
			;
		flag_motor = 0;
		switch (rkr) {
		case 0:
			GPIO_Write(MOTOR_GPIO, RMOTOR_PIN3);
			break;
		case 1:
			GPIO_Write(MOTOR_GPIO, RMOTOR_PIN2);
			break;
		case 2:
			GPIO_Write(MOTOR_GPIO, RMOTOR_PIN4);
			break;
		case 3:
			GPIO_Write(MOTOR_GPIO, RMOTOR_PIN1);
			break;
		default:
			rkr = 0;
			break;
		}
		if (++rkr > 3)
			rkr = 0;
		steps--;
	}
}

void forward(int steps) {

	while (steps) {
		while (!flag_motor)
			;
		flag_motor = 0;
		switch (kr) {
		case 0:
			GPIO_Write(MOTOR_GPIO, LMOTOR_PIN4| RMOTOR_PIN3);

			break;
		case 1:
			GPIO_Write(MOTOR_GPIO, LMOTOR_PIN2| RMOTOR_PIN2);

			break;
		case 2:
			GPIO_Write(MOTOR_GPIO, LMOTOR_PIN3| RMOTOR_PIN4);
			break;
		case 3:
			GPIO_Write(MOTOR_GPIO, LMOTOR_PIN1| RMOTOR_PIN1);
			break;
		default:
			kr = 0;
			break;
		}
		if (++kr > 3)
			kr = 0;
		steps--;
	}
}

void backward(int steps) {

	while (steps) {
		while (!flag_motor)
			;
		flag_motor = 0;
		switch (kr) {
		case 0:
			GPIO_Write(MOTOR_GPIO, RMOTOR_PIN4| LMOTOR_PIN3);

			break;
		case 1:
			GPIO_Write(MOTOR_GPIO, RMOTOR_PIN2| LMOTOR_PIN2);

			break;
		case 2:
			GPIO_Write(MOTOR_GPIO, RMOTOR_PIN3| LMOTOR_PIN4);
			break;
		case 3:
			GPIO_Write(MOTOR_GPIO, RMOTOR_PIN1| LMOTOR_PIN1);
			break;
		default:
			kr = 0;
			break;
		}
		if (++kr > 3)
			kr = 0;
		steps--;
	}
}




void motor_init() {
//GPIO CONFIG
	RCC_APB2PeriphClockCmd(
			RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
					| RCC_APB2Periph_GPIOD, ENABLE);

	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = LMOTOR_PIN1 | LMOTOR_PIN2 | LMOTOR_PIN3 | LMOTOR_PIN4
			| RMOTOR_PIN1 | RMOTOR_PIN2 | RMOTOR_PIN3 | RMOTOR_PIN4;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(MOTOR_GPIO, &gpio);

//TIMER CONFIG
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef tim;
	NVIC_InitTypeDef nvic;

	TIM_TimeBaseStructInit(&tim);
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Prescaler = 6400 - 1;
	tim.TIM_Period = MOTORDELAY - 1;
	TIM_TimeBaseInit(TIM2, &tim);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	TIM_Cmd(TIM2, ENABLE);



	nvic.NVIC_IRQChannel = TIM2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

}
