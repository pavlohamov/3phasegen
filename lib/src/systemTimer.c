/*
 * systemTimer.c
 *
 *  Created on: Jan 8, 2016
 *      Author: pavelgamov
 */


#include "system.h"
#include "systemTimer.h"
#include <stddef.h>
#include "timers.h"
#include "Queue.h"

#include "bsp.h"
#include "stm32f0xx_rcc.h"

static struct {
	uint32_t activeTime;
	uint32_t passiveTime;
} s_timing[] = {
		[INFORM_INIT] = { 0.1*BSP_TICKS_PER_SECOND, 0.3*BSP_TICKS_PER_SECOND },
		[INFORM_IDLE] = { 0.1*BSP_TICKS_PER_SECOND, BSP_TICKS_PER_SECOND },
		[INFORM_SLEEP] = { 0.05*BSP_TICKS_PER_SECOND, 2*BSP_TICKS_PER_SECOND},
		[INFORM_CONNECTION_LOST] = { 0.1*BSP_TICKS_PER_SECOND, 0.5*BSP_TICKS_PER_SECOND},
		[INFORM_ERROR] = { 0.05*BSP_TICKS_PER_SECOND, 0.05*BSP_TICKS_PER_SECOND},
};

static systemStatus_t s_systemStatus = INFORM_ERROR;
static uint32_t s_systemStatusTimer = 0;
static ledOutputControl_t s_systemLed = NULL;
static volatile uint32_t s_delayDecrement = 0;
static volatile uint32_t s_uptimeSeconds = 0;
static volatile uint32_t s_uptimeTicks = 0;

static struct {
	uint32_t sec;
	uint32_t msec;
} s_uptime;


void System_setStatus(systemStatus_t status) {

	if(status < INFORM_LAST) {
		s_systemStatus = status;
	}
}

void System_init(ledOutputControl_t control) {

	RCC_ClocksTypeDef RCC_ClockFreq;
	RCC_GetClocksFreq(&RCC_ClockFreq);
	SysTick_Config(RCC_ClockFreq.HCLK_Frequency / BSP_TICKS_PER_SECOND);

	System_setStatus(INFORM_INIT);
	s_systemLed = control;
}

void System_delayMsDummy(uint32_t delay) {
	uint32_t irq = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk);
	if(irq == 16 + SysTick_IRQn) // Could not wait if Called form SysTick isr
		return;
	s_delayDecrement = delay;
	while (s_delayDecrement);
}

uint32_t System_getUptime(void) {
	return s_uptimeSeconds;
}

uint32_t System_getUptimeMs(void) {
	return s_uptimeTicks % BSP_TICKS_PER_SECOND;
}



void SysTick_Handler(void);

void SysTick_Handler(void) {
	uint32_t period = s_timing[s_systemStatus].activeTime + s_timing[s_systemStatus].passiveTime;
	if (s_systemLed)
		s_systemLed(s_systemStatusTimer <= s_timing[s_systemStatus].activeTime);
	if (++s_systemStatusTimer > period) {
		s_systemStatusTimer = 0;
	}
	if (++s_uptime.msec >= BSP_TICKS_PER_SECOND) {
		s_uptime.msec = 0;
		s_uptime.sec++;
		EventQueue_Push(EVENT_SYSTICK, (void*)s_uptimeSeconds, NULL);
	}
	if (s_delayDecrement && s_delayDecrement--){};

	if (!(++s_uptimeTicks % BSP_TICKS_PER_SECOND)) {
		s_uptimeSeconds++;
	}
	Timer_makeTick();
}
