/*
 * bspGpio.c
 *
 *  Created on: Jan 19, 2017
 *      Author: shapa
 */

#include <stdlib.h>

#include "bsp.h"
#include "stm32f0xx_hal_gpio.h"

typedef struct {
    GPIO_TypeDef *const port;
	const GPIO_InitTypeDef setting;
} BspGpioConfig_t;

static const BspGpioConfig_t s_gpioConfig[] = {

    [BSP_Pin_UART_TX] = { GPIOA, { GPIO_PIN_9, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIO_AF1_USART1 } },
    [BSP_Pin_UART_RX] = { GPIOA, { GPIO_PIN_10, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIO_AF1_USART1 } },


    [BSP_Pin_PWM_1] = { GPIOB, { GPIO_PIN_1, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF1_TIM3 } },
    [BSP_Pin_PWM_2] = { GPIOA, { GPIO_PIN_7, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF1_TIM3 } },
    [BSP_Pin_PWM_3] = { GPIOA, { GPIO_PIN_6, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF1_TIM3 } },


    [BSP_Pin_POL_1] = { GPIOF, { GPIO_PIN_0, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, 0 } },
    [BSP_Pin_POL_2] = { GPIOA, { GPIO_PIN_5, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, 0 } },
    [BSP_Pin_POL_3] = { GPIOA, { GPIO_PIN_4, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, 0 } },

    [BSP_Pin_LedYellow] = { GPIOA, { GPIO_PIN_2, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, 0 } },
    [BSP_Pin_LedRed] = { GPIOA, { GPIO_PIN_3, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, 0 } },


    [BSP_Pin_Adc_0] = { GPIOA, { GPIO_PIN_0, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0 } },
    [BSP_Pin_Adc_1] = { GPIOA, { GPIO_PIN_1, GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0 } },
};

void BSP_InitGpio(void) {
	static const size_t size = sizeof(s_gpioConfig)/sizeof(*s_gpioConfig);
	for (size_t i = 0; i < size; i++)
	    HAL_GPIO_Init((GPIO_TypeDef*)s_gpioConfig[i].port, (GPIO_InitTypeDef*)&s_gpioConfig[i].setting);

//    GPIO_PinAFConfig((GPIO_TypeDef*)s_gpioConfig[BSP_Pin_UART_TX].port, GPIO_PinSource9, GPIO_AF_1);
//    GPIO_PinAFConfig((GPIO_TypeDef*)s_gpioConfig[BSP_Pin_UART_RX].port, GPIO_PinSource10, GPIO_AF_1);
//
//	GPIO_PinAFConfig((GPIO_TypeDef*)s_gpioConfig[BSP_Pin_PWM_1].port, GPIO_PinSource1, GPIO_AF_1);
//	GPIO_PinAFConfig((GPIO_TypeDef*)s_gpioConfig[BSP_Pin_PWM_2].port, GPIO_PinSource7, GPIO_AF_1);
//	GPIO_PinAFConfig((GPIO_TypeDef*)s_gpioConfig[BSP_Pin_PWM_3].port, GPIO_PinSource6, GPIO_AF_1);


	BSP_SetPinVal(BSP_Pin_POL_1, false);
	BSP_SetPinVal(BSP_Pin_POL_2, false);
	BSP_SetPinVal(BSP_Pin_POL_3, false);
}

void BSP_SetPinVal(const BSP_Pin_t pin, const _Bool state) {
	if (pin > BSP_Pin_Last)
		return;
	if (state)
		s_gpioConfig[pin].port->BSRR = s_gpioConfig[pin].setting.Pin;
	else
		s_gpioConfig[pin].port->BRR = s_gpioConfig[pin].setting.Pin;
}

_Bool BSP_GetPinVal(const BSP_Pin_t pin) {
	if (pin > BSP_Pin_Last)
		return false;
	return s_gpioConfig[pin].port->IDR & s_gpioConfig[pin].setting.Pin;
}

