#include "stm32f1xx_hal.h"
#include "gpio.h"


#define BLACK  1  
#define WHITE  0  

uint8_t Track_L1(void)
{
return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
}	

uint8_t Track_L2(void){
return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
}	
uint8_t Track_R1(void){
return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
}		
uint8_t Track_R2(void){
return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
}		