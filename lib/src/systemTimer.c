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

#include "stm32f0xx.h"

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

static systemStatus_t s_systemStatus = INFORM_INIT;
static uint32_t s_systemStatusTimer = 0;
static ledOutputControl_t s_systemLed = NULL;

static volatile struct {
	uint32_t sec;
	uint16_t msec;
} s_uptime;

void System_setStatus(systemStatus_t status) {

	if(status < INFORM_LAST) {
		s_systemStatus = status;
	}
}

void System_init(ledOutputControl_t control) {

	s_systemLed = control;
	SysTick_Config(HAL_RCC_GetHCLKFreq() / BSP_TICKS_PER_SECOND);

	System_setStatus(INFORM_INIT);
}

void System_delayMsDummy(uint32_t delay) {
	uint32_t irq = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk);
	if(irq == 16 + SysTick_IRQn) // Could not wait if Called form SysTick isr
		return;
	uint32_t sec = s_uptime.sec + delay / 1000;
	uint16_t msec = s_uptime.msec + delay % 1000;
	if (msec >= BSP_TICKS_PER_SECOND) {
		msec -= BSP_TICKS_PER_SECOND;
		++sec;
	}
	while (s_uptime.sec < sec);
	while (s_uptime.msec < msec);
}

uint32_t System_getUptime(void) {
	return s_uptime.sec;
}

uint32_t System_getUptimeMs(void) {
	return s_uptime.msec;
}

uint32_t HAL_GetTick(void) {
    return s_uptime.sec * BSP_TICKS_PER_SECOND + s_uptime.msec;
}

void SysTick_Handler(void);

void SysTick_Handler(void) {
	if (s_systemLed) {
		uint32_t period = s_timing[s_systemStatus].activeTime + s_timing[s_systemStatus].passiveTime;
		s_systemLed(s_systemStatusTimer <= s_timing[s_systemStatus].activeTime);
		if (++s_systemStatusTimer > period) {
			s_systemStatusTimer = 0;
		}
	}
	if (++s_uptime.msec >= BSP_TICKS_PER_SECOND) {
		s_uptime.msec = 0;
		s_uptime.sec++;
		EventQueue_Push(EVENT_SYSTICK, (void*)s_uptime.sec, NULL);
	}
	Timer_makeTick();
}
