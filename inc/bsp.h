/*
 * bsp.h
 *
 *  Created on: Jan 18, 2017
 *      Author: pavelgamov
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define BSP_TICKS_PER_SECOND 1000
#define MINUTE_TICKS (BSP_TICKS_PER_SECOND*60)

typedef enum {
	BSP_Pin_PWM_1,
	BSP_Pin_PWM_2,
	BSP_Pin_PWM_3,
	BSP_Pin_EN_1,
	BSP_Pin_EN_2,
	BSP_Pin_EN_3,
	BSP_Pin_LED,
	BSP_Pin_Adc,

	BSP_Pin_Last,
} BSP_Pin_t;


_Bool BSP_Init(void);
void BSP_InitGpio(void);

void BSP_FeedWatchdog(void);

void BSP_SetPinVal(const BSP_Pin_t pin, const _Bool state);
_Bool BSP_GetPinVal(const BSP_Pin_t pin);

#ifdef __cplusplus
}
#endif
