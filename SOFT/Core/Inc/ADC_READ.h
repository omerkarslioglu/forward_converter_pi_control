/*
 * ADC_READ.h
 *
 *  Created on: Jun 11, 2022
 *      Author: Omer Karslioglu
 */
#include "stm32f4xx_hal.h"
#include "main.h"

#ifndef INC_ADC_READ_H_
#define INC_ADC_READ_H_
#endif /* INC_ADC_READ_H_ */

ADC_HandleTypeDef hadc1;

void ADC_Select_CH0(void);
void ADC_Select_CH1(void);
void ADC_Select_CH2(void);

#define Avg_Slope .0025
#define V25 0.76



#ifdef __cplusplus
#endif /* __ADC_READ.h */

/************************ (C) COPYRIGHT Omer Karslioglu *****END OF FILE****/


