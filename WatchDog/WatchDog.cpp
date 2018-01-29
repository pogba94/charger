#include "WatchDog.h"

/*!
 * @brief watch dog initialization
 * @parme value of timeout, default value is 5000 ms
 * @return none
*/

void wDogInit(uint16_t time_ms)
{
	wdog_config_t config;

	WDOG_GetDefaultConfig(&config);
	config.timeoutValue = time_ms;    //timeout value 5s
	WDOG_Init((WDOG_Type*)WDOG_BASE,&config);
}

/*!
 * @brief feed dog
 * @parme none
 * @return none
*/

void wDogFeed(void)
{
	WDOG_Refresh((WDOG_Type*)WDOG_BASE);
}
