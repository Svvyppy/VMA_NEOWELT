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
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, speed);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, speed);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, speed);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, speed);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, speed);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed);

	HAL_Delay(4000);
}

void Thruster_Set_Speed(uint16_t* speed)
{

// 	data[0] = ((speed[0] << 8) | (speed[1] & 0xFF));
// 	data[1] = ((speed[2] << 8) | (speed[3] & 0xFF));
// 	data[2] = ((speed[4] << 8) | (speed[5] & 0xFF));
// 	data[3] = ((speed[6] << 8) | (speed[7] & 0xFF));
// 	data[4] = ((speed[8] << 8) | (speed[9] & 0xFF));
// 	data[5] = ((speed[10] << 8) | (speed[11] & 0xFF));
// 	data[6] = ((speed[12] << 8) | (speed[13] & 0xFF));
// 	data[7] = ((speed[14] << 8) | (speed[15] & 0xFF));
// 	data[8] = ((speed[16] << 8) | (speed[17] & 0xFF));
// 	data[9] = ((speed[18] << 8) | (speed[19] & 0xFF));
// 	data[10] = ((speed[20] << 8) | (speed[21] & 0xFF));
// 	data[11] = ((speed[22] << 8) | (speed[23] & 0xFF));


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



