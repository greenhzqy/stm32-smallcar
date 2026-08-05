#include "motor.h"

void motor_init(void)
{
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  
__HAL_TIM_ENABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_1);
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_2);
	
	motor_setspeed(0, 0);
	
	
}

void motor_setspeed(int16_t left_speed,int16_t right_speed)
{
	
	if( left_speed > 0 ){
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
	
	}

else if( left_speed < 0 ){
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
	right_speed=-right_speed;
	
	}
else{
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
     
	
}
	
	if( right_speed > 0 ){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);
	
	
	}
	else if( right_speed < 0 ){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);
	
	right_speed=-right_speed;
	
	}
else{
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);
     
	
}

//if ( left_speed>1000)
	//left_speed=1000;
//if ( right_speed>1000)
	//right_speed=1000;

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left_speed);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right_speed);


}