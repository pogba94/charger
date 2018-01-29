#ifndef WATCHDOG_H
#define WATCHDOG_H

#ifdef __cplusplus

extern "C" {
#endif
	#include "fsl_wdog.h"
#ifdef __cplusplus
}
#endif

void wDogInit(uint16_t);
void wDogFeed(void);

#endif

