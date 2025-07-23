/*
 * SERVO_interface.h
 *
 *  Created on: Jan 22, 2025
 *      Author: user
 */

#ifndef INC_SERVO_INTERFACE_H_
#define INC_SERVO_INTERFACE_H_

#include "main.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim1;

uint32_t AppMap(uint32_t InMin,uint32_t InMax,uint32_t OutMin,uint32_t OutMax,uint32_t InVal);

void SERVO_voidInit(TIM_HandleTypeDef tim,uint8_t Copy_u8Channel_ID);

void SERVO_voidRotate(TIM_HandleTypeDef tim,uint8_t Copy_u8Channel_ID,uint8_t Copy_u8Angle);

#endif /* INC_SERVO_INTERFACE_H_ */
