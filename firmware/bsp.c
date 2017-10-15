/*
 * bsp.c
 *
 *  Created on: May 18, 2016
 *      Author: shapa
 */

#include "bsp.h"
#include "system.h"
#include "systemTimer.h"
#include "Queue.h"
#include "timers.h"

#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_adc.h"
#include "stm32f0xx_iwdg.h"
#include "stm32f0xx_tim.h"

#include <stddef.h>

#define ADC_TIMEOUT (BSP_TICKS_PER_SECOND/50)

static void initialize_RCC(void);
static void initWdt(void);
static void initADC(void);
static void initADC_NVIC(void);
static void initPWM_TIM(void);
static void initPWM_OC(void);

static void setSystemLed(_Bool state);

static void onAdcTimeout(uint32_t id, void *data);
static _Bool handleAdcITFlag(uint32_t flag);

static uint32_t s_adcTimerId = INVALID_HANDLE;

_Bool BSP_Init(void) {
	initialize_RCC();
	initWdt();
	BSP_InitGpio();
	System_init(setSystemLed);
	System_setStatus(INFORM_IDLE);

	initADC();
	initPWM_TIM();
	initPWM_OC();

	return true;
}

void BSP_FeedWatchdog(void) {
	IWDG_ReloadCounter();
}

static void initialize_RCC(void) {

	RCC_HSEConfig(RCC_HSE_OFF);
	RCC_WaitForHSEStartUp(); // really we wait for shutdown
	RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div4);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	GPIO_DeInit(GPIOA);
	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOF);
}

static void initWdt(void) {
//	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
//	IWDG_SetPrescaler(IWDG_Prescaler_32);
//	IWDG_SetReload(0x0FFF);
//	IWDG_ReloadCounter();
//	IWDG_Enable();
}

static void initADC(void) {
	const static ADC_InitTypeDef iface = {
		ADC_Resolution_12b,
		DISABLE,
		ADC_ExternalTrigConvEdge_None,
		0,
		ADC_DataAlign_Right,
		ADC_ScanDirection_Upward
	};

	ADC_DeInit(ADC1);
	ADC_Init(ADC1, (ADC_InitTypeDef*)&iface);
	ADC_ClockModeConfig(ADC1, ADC_ClockMode_SynClkDiv4);
	ADC_ChannelConfig(ADC1, ADC_Channel_0, ADC_SampleTime_239_5Cycles);

	initADC_NVIC();
	ADC_ContinuousModeCmd(ADC1, DISABLE);
	ADC_DiscModeCmd(ADC1, ENABLE);

	ADC_ITConfig(ADC1, ADC_IT_ADRDY, ENABLE);
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

	ADC_GetCalibrationFactor(ADC1);
	ADC_Cmd(ADC1, ENABLE);

	s_adcTimerId = Timer_newArmed(ADC_TIMEOUT, true, onAdcTimeout, NULL);
}

static void initADC_NVIC(void) {
	NVIC_InitTypeDef nvic = {
			ADC1_IRQn,
			0,
			ENABLE
	};
	NVIC_Init(&nvic);
}

static void initPWM_TIM(void) {
	TIM_TimeBaseInitTypeDef iface = {
			0x2,
			TIM_CounterMode_Up,
			0xFF,
			TIM_CKD_DIV1,
			0
	};

	TIM_TimeBaseInit(TIM3, &iface);
	TIM_Cmd(TIM3, ENABLE);
}

static void initPWM_OC(void) {
    TIM_OCInitTypeDef pwm = {
                    TIM_OCMode_PWM1,
                    TIM_OutputState_Enable,
                    0,
                    0,
                    TIM_OCPolarity_High,
                    0, 0, 0
    };
    TIM_OC1Init(TIM3, &pwm);
    pwm.TIM_Pulse = 127;
    TIM_OC2Init(TIM3, &pwm);
    pwm.TIM_Pulse = 255;
    TIM_OC4Init(TIM3, &pwm);
    TIM_CCxCmd(TIM3, TIM_Channel_1, TIM_CCx_Enable);
    TIM_CCxCmd(TIM3, TIM_Channel_2, TIM_CCx_Enable);
    TIM_CCxCmd(TIM3, TIM_Channel_4, TIM_CCx_Enable);
}

static void setSystemLed(_Bool state) {
	BSP_SetPinVal(BSP_Pin_LED, state);
}

static void onAdcTimeout(uint32_t id, void *data) {
	Timer_disarm(id);
	ADC_ChannelConfig(ADC1, ADC_Channel_0, ADC_SampleTime_239_5Cycles);
	ADC_StartOfConversion(ADC1);
}

void ADC1_IRQHandler(void) {

	static const uint32_t adcITFlags[] = {
			ADC_IT_ADRDY,
			ADC_IT_EOSMP,
			ADC_IT_EOC,
			ADC_IT_EOSEQ,
			ADC_IT_OVR,
			ADC_IT_AWD,
	};
	static const size_t adcITFlagsSize = sizeof(adcITFlags)/(sizeof(*adcITFlags));

	for (size_t i = 0; i < adcITFlagsSize; i++)
		if (ADC_GetITStatus(ADC1, adcITFlags[i]) && handleAdcITFlag(adcITFlags[i]))
			ADC_ClearITPendingBit(ADC1, adcITFlags[i]);
}

static _Bool handleAdcITFlag(uint32_t flag) {
	_Bool clear = true;
	switch (flag) {
		case ADC_IT_ADRDY: {
			ADC_StartOfConversion(ADC1);
		} break;
		case ADC_IT_EOSMP: {
		} break;
		case ADC_IT_EOC: {
			uint32_t val = ADC_GetConversionValue(ADC1);
			EventQueue_Push(EVENT_ADC, (void*)val, NULL);
			Timer_rearm(s_adcTimerId);
		} break;
		case ADC_IT_EOSEQ: {
		} break;
		case ADC_IT_OVR: {
		} break;
		case ADC_IT_AWD: {
		} break;
		default: {
			clear = false;
		} break;
	}
	return clear;
}
