/*
 * delay_us.h
 *
 *  Created on:
 *      Author:
 */

#ifndef DELAY_US_H_
#define DELAY_US_H_

#include "stm32f4xx_hal.h"

uint32_t Get_Micros(void);
void Delay_Micros(uint16_t micros);


#endif /* DELAY_US_H_ */
