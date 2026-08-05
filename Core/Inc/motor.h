#ifndef __MOTOR_H
#define __MOTOR_H
#include "stm32f1xx_hal.h"  
#include "tim.h"           
void motor_init(void);     
void motor_setspeed(int16_t left_speed, int16_t right_speed);  // 设置左右电机速度

#endif