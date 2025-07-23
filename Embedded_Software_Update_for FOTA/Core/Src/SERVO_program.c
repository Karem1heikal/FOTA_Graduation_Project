/*
 * SERVO_program.c
 *
 *  Created on: Jan 23, 2025
 *      Author: user
 */
#include "SERVO_interface.h"
uint32_t AppMap(uint32_t InMin,uint32_t InMax,uint32_t OutMin,uint32_t OutMax,uint32_t InVal)
{
	uint32_t Local=0;
	Local = (((InVal-InMin)*(OutMax-OutMin))/(InMax-InMin))+OutMin;
	return Local;
}

void SERVO_voidInit(TIM_HandleTypeDef tim,uint8_t Copy_u8Channel_ID)
{
	HAL_TIM_PWM_Start(&tim, (uint32_t)Copy_u8Channel_ID);
}
void SERVO_voidRotate(TIM_HandleTypeDef tim,uint8_t Copy_u8Channel_ID,uint8_t Copy_u8Angle)
{
	uint16_t Local_u16Angle = 0;
	Local_u16Angle = AppMap( 0UL, 180UL,750UL, 2500UL,(uint32_t)Copy_u8Angle);
	__HAL_TIM_SET_COMPARE(&tim,Copy_u8Channel_ID,Local_u16Angle/2);
}


