#ifndef __TRACK_H
#define __TRACK_H

#include "stm32f1xx_hal.h"
#include "gpio.h"


#define BLACK  1  
#define WHITE  0  

uint8_t Track_L1(void);  
uint8_t Track_L2(void); 
uint8_t Track_R1(void);  
uint8_t Track_R2(void);  
#endif