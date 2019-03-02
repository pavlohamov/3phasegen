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

#define ADC_TIMEOUT (BSP_TICKS_PER_SECOND/50)

static inline void initialize_RCC(void);
static inline void initWdt(void);
static inline void initADC(void);
static inline void initADC_NVIC(void);
static inline void initPWM_TIM(void);
static inline void initPWM_OC(void);
static inline void initSIN_TIM(void);

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

    BSP_SetPinPWM(BSP_Pin_PWM_1, 0x7F);
	BSP_SetPinPWM(BSP_Pin_PWM_2, 0x7F);
    BSP_SetPinPWM(BSP_Pin_PWM_3, 0x7F);

    BSP_SetPinVal(BSP_Pin_POL_1, 0);
    BSP_SetPinVal(BSP_Pin_POL_2, 0);
    BSP_SetPinVal(BSP_Pin_POL_3, 0);

	return true;
}

void BSP_FeedWatchdog(void) {
	IWDG_ReloadCounter();
}

void BSP_SetPinPWM(const BSP_Pin_t pin, int32_t value) {

    value = abs(value);
	switch (pin) {
	case BSP_Pin_PWM_1:
		TIM3->CCR2 = value;
		break;
	case BSP_Pin_PWM_2:
		TIM3->CCR4 = value;
		break;
	case BSP_Pin_PWM_3:
		TIM3->CCR1 = value;
		break;
	default:
		return;
		break;
	}
}

void BSP_SetSinBase(const uint32_t value) {
	TIM_TimeBaseInitTypeDef iface = {
        value,
        TIM_CounterMode_Up,
        0xFF,
        TIM_CKD_DIV1,
        0
	};

	TIM_TimeBaseInit(TIM14, &iface);
}

static void initialize_RCC(void) {
	RCC_HSEConfig(RCC_HSE_OFF);
	RCC_WaitForHSEStartUp(); // really we wait for shutdown
	RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div4);

	RCC->AHBENR |= RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOF;
	RCC->APB2ENR |= RCC_APB2Periph_ADC1;
	RCC->APB1ENR |= RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM14;

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
		0x8F,
//		0x1FF,
		TIM_CounterMode_Up,
		0xFF,
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

static void initSIN_TIM(void) {
//	BSP_SetSinBase(0x7FFF);
    BSP_SetSinBase(0xFFF);
	TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM14, ENABLE);

	static const NVIC_InitTypeDef nvic = {
		TIM14_IRQn,
		0,
		ENABLE
	};
	NVIC_Init((NVIC_InitTypeDef*)&nvic);
}
static const int8_t phaseA[] = {
        1, 6, 12, 19, 25, 31, 37, 43, 49, 54, 60, 65, 71, 76, 81, 85, 90, 94, 98, 102, 106, 109, 112, 115, 117, 120, 122, 123, 125, 126, 126,
        127, 127, 127, 126, 126, 125, 123, 122, 120, 117, 115, 112, 109, 106, 102, 98, 94, 90, 85, 81, 76, 71, 65, 60, 54, 49, 43, 37, 31, 25, 19,
        12, 6, 1, -6, -12, -19, -25, -31, -37, -43, -49, -54, -60, -65, -71, -76, -81, -85, -90, -94, -98, -102, -106, -109, -112, -115, -117, -120, -122, -123, -125,
        -126, -126, -127, -127, -127, -126, -126, -125, -123, -122, -120, -117, -115, -112, -109, -106, -102, -98, -94, -90, -85, -81, -76, -71, -65, -60, -54, -49, -43, -37, -31,
        -25, -19, -12, -6,


};
static const int8_t phaseB[] = {
        110, 107, 103, 99, 95, 91, 87, 82, 77, 72, 67, 62, 56, 51, 45, 39, 33, 27, 21, 15, 8, 2, -4, -10, -17, -23, -29, -35, -41, -47, -52,
        -58, -63, -69, -74, -79, -84, -88, -93, -97, -101, -104, -108, -111, -114, -117, -119, -121, -123, -124, -125, -126, -127, -127, -127, -127, -126, -125, -124, -122, -120, -118,
        -116, -113, -110, -107, -103, -99, -95, -91, -87, -82, -77, -72, -67, -62, -56, -51, -45, -39, -33, -27, -21, -15, -8, -2, 4, 10, 17, 23, 29, 35, 41,
        47, 52, 58, 63, 69, 74, 79, 84, 88, 93, 97, 101, 104, 108, 111, 114, 117, 119, 121, 123, 124, 125, 126, 127, 127, 127, 127, 126, 125, 124, 122,
        120, 118, 116, 113,

};
static const int8_t phaseC[] = {
        -110, -113, -116, -118, -120, -122, -124, -125, -126, -127, -127, -127, -127, -126, -125, -124, -123, -121, -119, -117, -114, -111, -108, -104, -101, -97, -93, -88, -84, -79, -74,
        -69, -64, -58, -52, -47, -41, -35, -29, -23, -17, -10, -4, 2, 8, 15, 21, 27, 33, 39, 45, 51, 56, 62, 67, 72, 77, 82, 87, 91, 95, 99,
        103, 107, 110, 113, 116, 118, 120, 122, 124, 125, 126, 127, 127, 127, 127, 126, 125, 124, 123, 121, 119, 117, 114, 111, 108, 104, 101, 97, 93, 88, 84,
        79, 74, 69, 64, 58, 52, 47, 41, 35, 29, 23, 17, 10, 4, -2, -8, -15, -21, -27, -33, -39, -45, -51, -56, -62, -67, -72, -77, -82, -87, -91,
        -95, -99, -103, -107,

};
static const size_t stepsMax = sizeof(phaseA)/sizeof(*phaseA);

void TIM14_IRQHandler(void) {
	TIM_ClearFlag(TIM14, TIM_IT_Update);

	static size_t step = 0;

    const int32_t valA = phaseA[step];
    const int32_t valB = phaseB[step];
    const int32_t valC = phaseC[step];

//    if (!valA || !valB || !valC)
//        trace_printf("%d %d %d\n", valA, valB, valC);

    BSP_SetPinPWM(BSP_Pin_PWM_1, valA);
    BSP_SetPinPWM(BSP_Pin_PWM_2, valB);
    BSP_SetPinPWM(BSP_Pin_PWM_3, valC);

    BSP_SetPinVal(BSP_Pin_POL_1, valA > 0);
    BSP_SetPinVal(BSP_Pin_POL_2, valB > 0);
    BSP_SetPinVal(BSP_Pin_POL_3, valC > 0);

	if (++step >= stepsMax)
	    step = 0;
}

static inline void setSystemLed(_Bool state) {
	BSP_SetPinVal(BSP_Pin_LED, state);
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

