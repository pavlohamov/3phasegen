
#include <stdio.h>
#include <stdlib.h>

#include "bsp.h"
#include "Queue.h"
#include "timers.h"
#include "systemTimer.h"

#include "dbg_base.h"
#if 01
#include "dbg_trace.h"
#endif

static inline void onTimerPush(uint32_t id) {
	EventQueue_Push(EVENT_TIMCALL, (void*)id, NULL);
}
//#define PI 3.14159265
//void mappop(int size, int off) {
//    double angle = 360.0/size;
//    int line = 0;
//    for (int i = 0; i < size; ++i) {
//        double rad = (off + angle*i)*PI/180.0;
//        double val = std::sin(rad);
//        printf("%.0f, ", 127*val);
//        if (++line > 30) {
//            line = 0;
//            printf("\n");
//        }
//    }
//    printf("\n");
//    printf("\n");
//}
//int size = 128;
//mappop(size, 0);
//mappop(size, 120);
//mappop(size, 240);

int main(int argc, char* argv[]) {

	Timer_init(onTimerPush);
	BSP_Init();
//	while (System_getUptime() < 1 || System_getUptimeMs() < 500);
	BSP_SetSinBase(0x3);
	DBGMSG_INFO("System starting");
	while (true) {

		Event_t event;
		EventQueue_Pend(&event);
		BSP_FeedWatchdog();
		uint32_t intVal = (uint32_t)event.data;
		switch (event.type) {
			case EVENT_SYSTICK:
				break;
			case EVENT_TIMCALL:
				Timer_onTimerCb(intVal);
				break;
			case EVENT_ADC: {
//			    DBGMSG_H("adc %ld", intVal);
				BSP_SetSinBase(intVal);
				break;
			}
			default:
				break;
		}
		EventQueue_Dispose(&event);
	}
	return 0;
}
