#ifndef OTACHARGER_H
#define OTACHARGER_H

#include "UserConfig.h"

#ifdef CHARGING_CURRENT_16A
	#define VERSION_INFO  "WPI-EVCharger_16A_K64F_20180129_V0.5.6"
#endif

#ifdef CHARGING_CURRENT_32A
	#define VERSION_INFO  "WPI-EVCharger_32A_K64F_20180129_V0.5.6"
#endif

void OTAInit(void);
void updateCode(void);

#endif