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
#include "Trace.h"

#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_adc.h"
#include "stm32f0xx_iwdg.h"
#include "stm32f0xx_tim.h"

#include <stddef.h>
#include <stdlib.h>
#include <math.h>

#define ADC_TIMEOUT (BSP_TICKS_PER_SECOND/50)

static inline void initialize_RCC(void);
static inline void initWdt(void);
static inline void initADC(void);
static inline void initADC_NVIC(void);
static inline void initPWM_TIM(void);
static inline void initPWM_OC(void);
static inline void initSIN_TIM(void);
static inline void setPwm(const BSP_Pin_t pin, int32_t value);

static void setSystemLed(_Bool state);

static void onAdcTimeout(uint32_t id, void *data);

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
	initSIN_TIM();

    setPwm(BSP_Pin_PWM_1, 0x7E);
	setPwm(BSP_Pin_PWM_2, 0x7E);
    setPwm(BSP_Pin_PWM_3, 0x7E);

    BSP_SetPinVal(BSP_Pin_POL_1, 0);
    BSP_SetPinVal(BSP_Pin_POL_2, 0);
    BSP_SetPinVal(BSP_Pin_POL_3, 0);

//	BSP_SetSinBase(0x7FFF);
	return true;
}

void BSP_FeedWatchdog(void) {
	IWDG_ReloadCounter();
}

void BSP_SetSinBase(const uint32_t value) {
	TIM_TimeBaseInitTypeDef iface = {
        value,
        TIM_CounterMode_Up,
        0xFF,
        TIM_CKD_DIV1,
        0
	};

	TIM_Cmd(TIM14, DISABLE);
	TIM_TimeBaseInit(TIM14, &iface);
	TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM14, ENABLE);
}

static void initialize_RCC(void) {
	RCC_HSEConfig(RCC_HSE_OFF);
	RCC_WaitForHSEStartUp(); // really we wait for shutdown
	RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div4);

	RCC->AHBENR |= RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOF;
	RCC->APB1ENR |= RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM14;
    RCC->APB2ENR |= RCC_APB2Periph_ADC1 | RCC_APB2Periph_USART1;

	RCC->AHBRSTR |= RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOF;
    RCC->AHBRSTR &= ~(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOF);

    RCC->APB2RSTR |= RCC_APB2Periph_ADC1;
    RCC->APB2RSTR &= ~RCC_APB2Periph_ADC1;
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

	ADC_Init(ADC1, (ADC_InitTypeDef*)&iface);
	ADC_ClockModeConfig(ADC1, ADC_ClockMode_SynClkDiv4);
	ADC_ChannelConfig(ADC1, ADC_Channel_0, ADC_SampleTime_239_5Cycles);

	initADC_NVIC();
	ADC_ContinuousModeCmd(ADC1, DISABLE);
	ADC_DiscModeCmd(ADC1, ENABLE);

	ADC_ITConfig(ADC1, ADC_IT_ADRDY | ADC_IT_EOC, ENABLE);

	ADC_GetCalibrationFactor(ADC1);
	ADC_Cmd(ADC1, ENABLE);

	s_adcTimerId = Timer_newArmed(ADC_TIMEOUT, true, onAdcTimeout, NULL);
}

static void initADC_NVIC(void) {
	static const NVIC_InitTypeDef nvic = {
        ADC1_IRQn,
        0,
        ENABLE
	};
	NVIC_Init((NVIC_InitTypeDef*)&nvic);
}

static void initPWM_TIM(void) {
	static const TIM_TimeBaseInitTypeDef iface = {
		0x07,
		TIM_CounterMode_Up,
		0x7E,
		TIM_CKD_DIV1,
		0
	};

	TIM_TimeBaseInit(TIM3, (TIM_TimeBaseInitTypeDef*)&iface);
	TIM_Cmd(TIM3, ENABLE);
}

static void initPWM_OC(void) {

    static const TIM_OCInitTypeDef pwm = {
        TIM_OCMode_PWM1,
		TIM_OutputState_Enable,
		TIM_OutputNState_Enable,
		0,
		TIM_OCPolarity_High,
		TIM_OCNPolarity_Low,
		TIM_OCIdleState_Set,
		TIM_OCNIdleState_Reset
    };
    TIM_OC1Init(TIM3, (TIM_OCInitTypeDef*)&pwm);
    TIM_OC2Init(TIM3, (TIM_OCInitTypeDef*)&pwm);
    TIM_OC4Init(TIM3, (TIM_OCInitTypeDef*)&pwm);

    TIM3->CCER |= (TIM_CCx_Enable << TIM_Channel_1) | (TIM_CCx_Enable << TIM_Channel_2) | (TIM_CCx_Enable << TIM_Channel_4);
}

static inline void setPwm(const BSP_Pin_t pin, int32_t value) {

    if (value < 0) {
    	value = -value;
    }
	switch (pin) {
	case BSP_Pin_PWM_1:
		TIM3->CCR4 = value;
		break;
	case BSP_Pin_PWM_2:
		TIM3->CCR2 = value;
		break;
	case BSP_Pin_PWM_3:
		TIM3->CCR1 = value;
		break;
	default:
		return;
		break;
	}
}

static void initSIN_TIM(void) {
	static const NVIC_InitTypeDef nvic = {
		TIM14_IRQn,
		0,
		ENABLE
	};
	NVIC_Init((NVIC_InitTypeDef*)&nvic);
}

static const int8_t phaseA[] = {
	0, 39, 75, 103, 121, 127, 121, 103, 75, 39, 0, -39, -75, -103, -121, -127, -121, -103, -75, -39,
};
static const int8_t phaseB[] = {
	110, 85, 52, 13, -26, -64, -94, -116, -126, -124, -110, -85, -52, -13, 26, 63, 94, 116, 126, 124,
};
static const int8_t phaseC[] = {
	-110, -124, -126, -116, -94, -64, -26, 13, 52, 85, 110, 124, 126, 116, 94, 63, 26, -13, -52, -85,

};
static const size_t stepsMax = sizeof(phaseA)/sizeof(*phaseA);

void TIM14_IRQHandler(void) {

	static size_t step = 0;

    const int32_t valA = phaseA[step];
    const int32_t valB = phaseB[step];
    const int32_t valC = phaseC[step];

//    if (!valA || !valB || !valC)
//        trace_printf("%d %d %d\n", valA, valB, valC);

    setPwm(BSP_Pin_PWM_1, valA);
    setPwm(BSP_Pin_PWM_2, valB);
    setPwm(BSP_Pin_PWM_3, valC);

    BSP_SetPinVal(BSP_Pin_POL_1, valA > 0);
    BSP_SetPinVal(BSP_Pin_POL_2, valB > 0);
    BSP_SetPinVal(BSP_Pin_POL_3, valC > 0);

	if (++step >= stepsMax)
	    step = 0;
	TIM_ClearFlag(TIM14, TIM_IT_Update);
}

static inline void setSystemLed(_Bool state) {
	BSP_SetPinVal(BSP_Pin_LedRed, state);
}

static void onAdcTimeout(uint32_t id, void *data) {
	Timer_disarm(id);
	ADC_ChannelConfig(ADC1, ADC_Channel_0, ADC_SampleTime_239_5Cycles);
	ADC_StartOfConversion(ADC1);
}

void ADC1_IRQHandler(void) {
	if (ADC_GetITStatus(ADC1, ADC_IT_ADRDY)) {
		ADC_StartOfConversion(ADC1);
		ADC_ClearITPendingBit(ADC1, ADC_IT_ADRDY);
	}

	if (ADC_GetITStatus(ADC1, ADC_IT_EOC)) {
		uint32_t val = ADC_GetConversionValue(ADC1);
		EventQueue_Push(EVENT_ADC, (void*)val, NULL);
		Timer_rearm(s_adcTimerId);
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
	}
}

