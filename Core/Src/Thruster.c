#include "Thruster.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;



void Thruster_Init()
{
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4);


	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_4);


	Thruster_Stop();

}

void Thruster_Stop()
{
	int speed = 150;

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);

	HAL_Delay(4000);
}

void Thruster_Set_Speed(uint16_t* speed)
{

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed[0]);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed[1]);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed[2]);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed[3]);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed[4]);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed[5]);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, speed[6]);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, speed[7]);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed[8]);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed[9]);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed[10]);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed[11]);
}


void Process_data(uint16_t *speed){
	if((speed[0] == 0xFF) && (speed[1] == 0xFD)){
		Thruster_Set_Speed(speed +2);
	}
}
