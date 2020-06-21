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


#include "dbg_base.h"
#if 01
#include "dbg_trace.h"
#endif

//#include "stm32f0xx_hal_def.h"

#include "stm32f0xx_hal_rcc.h"
//#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_hal_iwdg.h"
#include "stm32f0xx_hal_usart.h"
#include "stm32f0xx_hal_adc.h"
#include "stm32f0xx_hal_tim.h"

#include <stddef.h>
#include <stdlib.h>
#include <math.h>

#define ADC_TIMEOUT (BSP_TICKS_PER_SECOND/40)

static inline void initialize_RCC(void);
static inline void initWdt(void);
static inline void initUart(void);
static inline void initADC(void);
static inline void initPWM_TIM(void);
static inline void initSIN_TIM(void);
static inline void setPwm(const BSP_Pin_t pin, int32_t value);

static void setSystemLed(_Bool state);

static void onAdcTimeout(uint32_t id, void *data);

typedef struct {
    int8_t *a;
    int8_t *b;
    int8_t *c;
    uint16_t presc;
    size_t count;
} PhaseCfg_t;

static struct {
    bool started;
    bool pending;
    PhaseCfg_t cfg;
} s_phase;

static struct {
    IWDG_HandleTypeDef wdt;
    USART_HandleTypeDef usart;
    TIM_HandleTypeDef pwmTim;
    TIM_HandleTypeDef sinTim;
    ADC_HandleTypeDef adc;
    uint32_t adcTim;
    struct {
        uint16_t adcBuff[10];
        int8_t adcCur;
    };
} s_bsp = {
    .wdt = { IWDG, .Init = { IWDG_PRESCALER_4, 0x07FF, IWDG_WINDOW_DISABLE } },
    .pwmTim = { TIM3, .Init = { 0x07, TIM_COUNTERMODE_UP, 0x7E, TIM_CLOCKDIVISION_DIV1, 0, TIM_AUTORELOAD_PRELOAD_DISABLE, } },
    .usart = { USART1, { 921600, USART_WORDLENGTH_8B, USART_STOPBITS_1, USART_PARITY_NONE, USART_MODE_TX, USART_POLARITY_LOW, USART_PHASE_1EDGE, USART_LASTBIT_DISABLE, } },
    .sinTim = { TIM14, .Init = { 0xFF, TIM_COUNTERMODE_UP, 0xFF, TIM_CLOCKDIVISION_DIV1, 0, TIM_AUTORELOAD_PRELOAD_DISABLE, } },
    .adc = { ADC1, .Init = { ADC_CLOCK_SYNC_PCLK_DIV2, ADC_RESOLUTION_12B, ADC_DATAALIGN_RIGHT, ADC_SCAN_DIRECTION_FORWARD, ADC_EOC_SINGLE_CONV, DISABLE, DISABLE,
                            .ContinuousConvMode = DISABLE,
                            .DiscontinuousConvMode = DISABLE,
                            .ExternalTrigConv = ADC_SOFTWARE_START,
                            .ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING,
                            .DMAContinuousRequests = DISABLE,
                            .Overrun = ADC_OVR_DATA_OVERWRITTEN,
                            .SamplingTimeCommon = ADC_SAMPLETIME_239CYCLES_5} },
    .adcTim = INVALID_HANDLE,
};


_Bool BSP_Init(void) {
	initialize_RCC();
	initWdt();
	BSP_InitGpio();
	initUart();
	System_init(setSystemLed);
	System_setStatus(INFORM_IDLE);

	initADC();
	initPWM_TIM();
	initSIN_TIM();

    setPwm(BSP_Pin_PWM_1, 0x7E);
	setPwm(BSP_Pin_PWM_2, 0x7E);
    setPwm(BSP_Pin_PWM_3, 0x7E);

    BSP_SetPinVal(BSP_Pin_POL_1, 0);
    BSP_SetPinVal(BSP_Pin_POL_2, 0);
    BSP_SetPinVal(BSP_Pin_POL_3, 0);

//	BSP_SetSinBase(0x7FFF);
    DBGMSG_INFO("SysClock %ld", HAL_RCC_GetSysClockFreq());
    DBGMSG_INFO("    HCLK %ld", HAL_RCC_GetHCLKFreq());
    DBGMSG_INFO("   PCLK1 %ld", HAL_RCC_GetPCLK1Freq());
	return true;
}

void BSP_FeedWatchdog(void) {
    HAL_IWDG_Refresh(&s_bsp.wdt);
}

#define PI 3.14159265
void fillBuffer(int8_t *buff, int size, int off) {
    double angle = 360.0/size;
    for (int i = 0; i < size; ++i) {
        double rad = (off + angle*i)*PI/180.0;
        double val = sin(rad);
        buff[i] = 127 * val;
    }
}

void BSP_SetSinBase(const uint32_t frequency) {
    static uint32_t lastFreq;
    if (!frequency || lastFreq == frequency)
        return;
    lastFreq = frequency;
    int frq = HAL_RCC_GetPCLK1Freq() / frequency;
    int period = 128;
    int elements = 128;
    int prescaller = 0;
    do {
        prescaller = frq / period / elements;
        if (prescaller)
            break;
        if (elements > 6) {
            elements--;
            continue;
        }
        if (period > 2) {
            period /= 2;
            elements = 128;
            if (!period)
                return;

            continue;
        }
    } while (1);
    DBGMSG_H("frequency %ld hz - period %d el %d prsc %d", frequency, period, elements, prescaller);
    PhaseCfg_t cfg = { .presc = prescaller, .count = elements };
    cfg.a = malloc(elements);
    if (!cfg.a)
        return;
    cfg.b = malloc(elements);
    if (!cfg.b) {
        free(cfg.a);
        return;
    }
    cfg.c = malloc(elements);
    if (!cfg.c) {
        free(cfg.a);
        free(cfg.b);
        return;
    }
    fillBuffer(cfg.a, elements, 0);
    fillBuffer(cfg.b, elements, 120);
    fillBuffer(cfg.c, elements, 240);
    if (!s_phase.started) {
        s_phase.cfg = cfg;
        s_phase.started = true;
        HAL_TIM_Base_Start_IT(&s_bsp.sinTim);
        return;
    }
    while (s_phase.pending)
        __WFI();
    s_phase.cfg = cfg;
    s_phase.pending = true;

}

static void initialize_RCC(void) {
    __HAL_RCC_HSE_CONFIG(RCC_HSE_OFF);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM14_CLK_ENABLE();

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DBGMCU_CLK_ENABLE();

    __HAL_RCC_ADC1_CLK_ENABLE();
}

static void initWdt(void) {
    HAL_IWDG_Init(&s_bsp.wdt);
    __HAL_IWDG_START(&s_bsp.wdt);
    HAL_IWDG_Refresh(&s_bsp.wdt);
}

static inline void initUart(void) {
    HAL_USART_Init(&s_bsp.usart);
}

static void initADC(void) {
    HAL_ADC_Init(&s_bsp.adc);
    HAL_NVIC_EnableIRQ(ADC1_IRQn);
	s_bsp.adcTim = Timer_newArmed(ADC_TIMEOUT, true, onAdcTimeout, NULL);
}

static void initPWM_TIM(void) {
    static const TIM_OC_InitTypeDef pwm = {
        TIM_OCMODE_PWM1,
        0x3F,
        TIM_OCPOLARITY_HIGH,
        TIM_OCNPOLARITY_LOW,
        TIM_OCFAST_ENABLE,
        TIM_OCIDLESTATE_RESET,
        TIM_OCNIDLESTATE_SET,
    };

    HAL_TIM_PWM_Init(&s_bsp.pwmTim);
    HAL_TIM_PWM_ConfigChannel(&s_bsp.pwmTim, (TIM_OC_InitTypeDef*)&pwm, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&s_bsp.pwmTim, (TIM_OC_InitTypeDef*)&pwm, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&s_bsp.pwmTim, (TIM_OC_InitTypeDef*)&pwm, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&s_bsp.pwmTim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&s_bsp.pwmTim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&s_bsp.pwmTim, TIM_CHANNEL_4);

}

static inline void setPwm(const BSP_Pin_t pin, int32_t value) {

    if (value < 0) {
    	value = -value;
    }
	switch (pin) {
	case BSP_Pin_PWM_1:
	    __HAL_TIM_SET_COMPARE(&s_bsp.pwmTim, TIM_CHANNEL_4, value);
		break;
	case BSP_Pin_PWM_2:
        __HAL_TIM_SET_COMPARE(&s_bsp.pwmTim, TIM_CHANNEL_2, value);
		break;
	case BSP_Pin_PWM_3:
        __HAL_TIM_SET_COMPARE(&s_bsp.pwmTim, TIM_CHANNEL_1, value);
		break;
	default:
		return;
		break;
	}
}

static void initSIN_TIM(void) {
    HAL_TIM_Base_Init(&s_bsp.sinTim);
    HAL_NVIC_EnableIRQ(TIM14_IRQn);
}

void TIM14_IRQHandler(void) {

    static PhaseCfg_t cfg;
	static size_t step = 0;
	if (s_phase.pending) {
	    step = s_phase.cfg.count * step / cfg.count;
        if (s_phase.cfg.presc != cfg.presc)
            __HAL_TIM_SET_PRESCALER(&s_bsp.sinTim, s_phase.cfg.presc);
        free(cfg.a);
        free(cfg.b);
        free(cfg.c);
	    cfg = s_phase.cfg;
	    s_phase.pending = false;
	}

    const int32_t valA = cfg.a[step];
    const int32_t valB = cfg.b[step];
    const int32_t valC = cfg.c[step];

//    if (!valA || !valB || !valC)
//        trace_printf("%d %d %d\n", valA, valB, valC);

    setPwm(BSP_Pin_PWM_1, valA);
    setPwm(BSP_Pin_PWM_2, valB);
    setPwm(BSP_Pin_PWM_3, valC);

    BSP_SetPinVal(BSP_Pin_POL_1, valA > 0);
    BSP_SetPinVal(BSP_Pin_POL_2, valB > 0);
    BSP_SetPinVal(BSP_Pin_POL_3, valC > 0);

    step = (step + 1) % cfg.count;
	__HAL_TIM_CLEAR_FLAG(&s_bsp.sinTim, TIM_FLAG_UPDATE);
}

static inline void setSystemLed(_Bool state) {
	BSP_SetPinVal(BSP_Pin_LedRed, state);
}

static void onAdcTimeout(uint32_t id, void *data) {
    Timer_disarm(id);
    ADC_ChannelConfTypeDef cfg = {
        ADC_CHANNEL_0,
        ADC_RANK_CHANNEL_NUMBER,
        0
    };
    HAL_ADC_ConfigChannel(&s_bsp.adc, &cfg);
    HAL_ADC_Start_IT(&s_bsp.adc);
}

static inline int adc_raw_to_millivolts(int32_t ref_mv, uint8_t resolution, int32_t val) {
    int32_t adc_mv = val * ref_mv;
    return (adc_mv >> resolution);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    static const int size = sizeof(s_bsp.adcBuff)/sizeof(*s_bsp.adcBuff);
    s_bsp.adcBuff[s_bsp.adcCur++] = HAL_ADC_GetValue(hadc);
    if (s_bsp.adcCur >= size)
         s_bsp.adcCur = 0;
    uint32_t val = 0;
    for (int i = 0; i < size; ++i)
        val += s_bsp.adcBuff[i];
    val /= size;
    val = adc_raw_to_millivolts(3300, 12, val);
    EventQueue_Push(EVENT_ADC, (void*)val, NULL);
    Timer_rearm(s_bsp.adcTim);
}
void ADC1_IRQHandler(void) {
    HAL_ADC_IRQHandler(&s_bsp.adc);
}

int BSP_write(const void *ptr, size_t size) {
    HAL_USART_Transmit(&s_bsp.usart, (uint8_t*)ptr, size, 0xFFFF);
    return size;
}


