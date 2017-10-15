/*
 * system.h
 *
 *  Created on: Dec 7, 2016
 *      Author: shapa
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* locks isr. Critical section entry. return isr state before call */
int System_Lock(void);

/* Unlocks isr. Critical section exit. accept isr state before crit section entry */
void System_Unlock(int primask);

/* Wait for interrupt/event */
void System_Poll(void);

#ifdef __cplusplus
}
#endif
