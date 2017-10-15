/*
 * bspGpio.c
 *
 *  Created on: Jan 19, 2017
 *      Author: shapa
 */

#include <stdlib.h>

#include "bsp.h"
#include "stm32f0xx_gpio.h"

typedef struct {
	const GPIO_TypeDef *port;
	const GPIO_InitTypeDef setting;
} BspGpioConfig_t;

static const BspGpioConfig_t s_gpioConfig[] = {

	[BSP_Pin_PWM_1] = { GPIOB, { GPIO_Pin_1, GPIO_Mode_AF,
		GPIO_Speed_Level_1, GPIO_OType_PP,  GPIO_PuPd_NOPULL} },

	[BSP_Pin_PWM_2] = { GPIOA, { GPIO_Pin_7, GPIO_Mode_AF,
		GPIO_Speed_Level_1, GPIO_OType_PP,  GPIO_PuPd_NOPULL} },

	[BSP_Pin_PWM_3] = { GPIOA, { GPIO_Pin_6, GPIO_Mode_AF,
		GPIO_Speed_Level_1, GPIO_OType_PP,  GPIO_PuPd_NOPULL} },


	[BSP_Pin_EN_1] = { GPIOA, { GPIO_Pin_9, GPIO_Mode_OUT,
			GPIO_Speed_Level_1, GPIO_OType_PP,  GPIO_PuPd_NOPULL} },

	[BSP_Pin_EN_2] = { GPIOA, { GPIO_Pin_10, GPIO_Mode_OUT,
			GPIO_Speed_Level_1, GPIO_OType_PP,  GPIO_PuPd_NOPULL} },

	[BSP_Pin_EN_3] = { GPIOA, { GPIO_Pin_5, GPIO_Mode_OUT,
				GPIO_Speed_Level_1, GPIO_OType_PP,  GPIO_PuPd_NOPULL} },


	[BSP_Pin_LED] = { GPIOF, { GPIO_Pin_0, GPIO_Mode_OUT,
				GPIO_Speed_Level_1, GPIO_OType_PP,  GPIO_PuPd_NOPULL} },

	[BSP_Pin_Adc] = { GPIOA, { GPIO_Pin_0, GPIO_Mode_IN,
			GPIO_Speed_Level_1, GPIO_OType_PP,  GPIO_PuPd_NOPULL} },

};

void BSP_InitGpio(void) {
	static const size_t size = sizeof(s_gpioConfig)/sizeof(*s_gpioConfig);
	for (size_t i = 0; i < size; i++)
		GPIO_Init((GPIO_TypeDef*)s_gpioConfig[i].port, (GPIO_InitTypeDef*)&s_gpioConfig[i].setting);

	GPIO_PinAFConfig((GPIO_TypeDef*)s_gpioConfig[BSP_Pin_PWM_1].port, GPIO_PinSource1, GPIO_AF_1);
	GPIO_PinAFConfig((GPIO_TypeDef*)s_gpioConfig[BSP_Pin_PWM_2].port, GPIO_PinSource7, GPIO_AF_1);
	GPIO_PinAFConfig((GPIO_TypeDef*)s_gpioConfig[BSP_Pin_PWM_3].port, GPIO_PinSource6, GPIO_AF_1);

	BSP_SetPinVal(BSP_Pin_EN_1, true);
	BSP_SetPinVal(BSP_Pin_EN_2, true);
	BSP_SetPinVal(BSP_Pin_EN_3, true);
}

void BSP_SetPinVal(const BSP_Pin_t pin, const _Bool state) {
	if (pin > BSP_Pin_Last)
		return;
	BitAction act = state ? Bit_SET : Bit_RESET;
	GPIO_WriteBit((GPIO_TypeDef*)s_gpioConfig[pin].port, s_gpioConfig[pin].setting.GPIO_Pin, act);
}

_Bool BSP_GetPinVal(const BSP_Pin_t pin) {
	if (pin > BSP_Pin_Last)
		return false;
	return GPIO_ReadInputDataBit((GPIO_TypeDef*)s_gpioConfig[pin].port, s_gpioConfig[pin].setting.GPIO_Pin);
}

