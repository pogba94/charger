#ifndef OTACHARGER_H
#define OTACHARGER_H

#include "UserConfig.h"

#ifdef CHARGING_CURRENT_16A
	#define VERSION_INFO  "WPI-EVCharger_16A_K64F_20180316_V0.6.4"
#endif

#ifdef CHARGING_CURRENT_32A
	#define VERSION_INFO  "WPI-EVCharger_16A_K64F_20180316_V0.6.4"
#endif

void OTAInit(void);
void updateCode(void);

#endif