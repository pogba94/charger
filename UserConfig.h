/*
* Note:last modification by orange cai in 20170328
*/

#ifndef USERCONFIG_H
#define USERCONFIG_H

#include "mbed.h"
#include "rtos.h"  //ericyang 20160718
#include "rtc_api.h" //orangecai 20170619

#include "EthernetInterface.h"
#include "flashLayout.h"

/* ------------------------------ Charging mode select ---------------------------*/
//#define SINGLE_CHIP_MODE            // ericyang 20161208 no network version, no need anyone to control, auto charging,discharging, just for single device mode
#ifdef SINGLE_CHIP_MODE
	#define NOT_CHECK_NETWORK //ericyang 20160921
	#define AUTO_CHARGING   //ericyang 20161208
#endif

#define TEST_MODE
#ifdef TEST_MODE
#ifndef NOT_CHECK_NETWORK
//#define AUTOTEST_SOCKET_JSON_MSG // ericyang 20161207
#endif
//#define TEST_NO_485   // ericyang 20161229
#endif

#define GB18487_1_2015_AUTH_FUNC   // ericyang 20170111   // ericyang 20161207

#define DEBUG_MODE  1       //when in debug mode,the device connect to another server port:22222 

/*----------------------- Charging exception handler configure -----------------------*/
#define CHARGING_EXCEPTION_HANDLE_ENABLE   //orangeCai 20170525

#ifdef CHARGING_EXCEPTION_HANDLE_ENABLE
	#define MAX_EXCEPTION_TIME_ALLOWED       30U
	#define NORMAL_TIME_HOLDED               10U
	#define RECHARGING_WAIT_TIMES_ALLOWED    3U
#endif
#define MAX_PAUSE_TIME                   180U  //the max paused time allowed while charging,unit:s
#define MAX_WAIT_TIME                    3600U //the max wait time for s2 switch on,unit:s           

/*------------------------------- Charging current select ----------------------------*/
#define PWM_DUTY_CURRENT_16A (0.266667)
#define PWM_DUTY_CURRENT_32A  (0.53333)

#define CHARGING_CURRENT_16A

#ifdef CHARGING_CURRENT_32A
	#undef CHARGING_CURRENT_16A 
#endif

#ifdef CHARGING_CURRENT_16A
  #undef CHARGING_CURRENT_32A
#endif

/* ---------------------------------- Function select----------------------------------*/
//#define LCD_ENABLE  //ericyang 20160713
#define LED_INFO_ENABLE  // ericyang 20161226
//#define RTC_ENABLE       //orangecai 20170619
//#define RTC_TEST
//#define WDOG_ENABLE      //orangecai 20170630
#define EEPROM_ENABLE
#ifdef WDOG_ENABLE
	#define  WDOG_TIMEOUT_VALUE_MS     20000     //define timeout value of watchdog  
#endif

#define DISABLE_NETWORK_CONMUNICATION_FUNC    // 20161107,stop communication with server for a few time

#define ENABLE_FULL_CHARGING_DETECT      //orangecai
#ifdef  ENABLE_FULL_CHARGING_DETECT
	#define  FULL_CHARGING_DETECT_TIME     60      //unit:s
#endif	
#define  NETWORK_COUNT_ENABLE
/* ---------------------------------- Socket configure --------------------------------- */
#define SOCKET_OUT_BUFFER_SIZE 256
#define SOCKET_IN_BUFFER_SIZE (4096+256)
#define SOCKET_RESEND_TIME 5  // if msg send not success, msg will be resend after 5s 

/* ----------------------------------- Type definitin ------------------------------------*/
typedef struct _EventFlag {
    bool heatbeatFlag;
    bool checkMeterFlag;
    bool updateChargerInfoFlag;
    bool updateChargerStatusFlag;  // ericyang 20160824
	  bool updateVersionDoneFlag;    // orangeCai 20170328
    bool updataVersionFailFlag;    //orangecai 20170720  
//   bool updateConnectStatusFlag;
    bool stopChargingFlag;
//   bool checkLatestVersionFromServerFlag; // ericyang 20160914
    bool getLatestFWFromServerFlag; // ericyang 20160914
    #ifdef DISABLE_NETWORK_CONMUNICATION_FUNC
    bool stopCommunicationFlag;  //ericyang 20161107 stop communication between sever and client for 1 min?
    #endif
	  bool firstConnectFlag;  //orange 20171213
} SystemEventHandle;

typedef struct _SocketInfo {
    char inBuffer[SOCKET_IN_BUFFER_SIZE];
    char outBuffer[SOCKET_OUT_BUFFER_SIZE];
} SocketInfo;

typedef enum _cpSignal {
    PWMNone = 0,
    PWM12V = 1,
    PWM9V = 2,
    PWM6V = 3,
		PWM0V = 4,
		PWMOtherVoltage = 5,
} CPSignal;

#ifdef GB18487_1_2015_AUTH_FUNC   // ericyang 20170111
typedef enum {
	//EV_DISCONNECT = 1,
	EV_IDLE = 2,
	EV_CONNECTED_PRE_START_CHARGING = 3,
	EV_CONNECTED_ON_CHARGING = 4,
	EV_CONNECTED_PRE_STOP_CHARGING = 5,
//	EV_CONNECTED_END_CHARGING = 6,
}EVChargingState;
#endif


/* ------------------------ Global variables and functions ------------------------ */
extern Serial pc;
#ifdef EEPROM_ENABLE
extern I2C i2c1;
#endif 
extern TCPSocketConnection tcpsocket;
extern char EVChargerModelNO[];
extern DigitalOut switchRelayOnoff;
extern Mutex myMutex;
extern int systemTimer;
extern SystemEventHandle eventHandle;
extern SocketInfo socketInfo;
extern float startEnergyReadFromMeter;
extern float totalEnergyReadFromMeter;
extern bool pwmState;
void disableCPPWM(void);
void enableCPPWM(float duty);
int getMeterInfo(void);
void getTimeStampMs(char* timeStampMs);
#ifdef RTC_ENABLE
void rtcInit(uint32_t timeSeconds);
#define RTC_INIT_TIME          (uint32_t)(3600*24*(365*47+30*10))
#endif
extern char tempBuffer[];
extern char dataBuffer[];

extern DigitalOut red;        //debug led
extern DigitalOut green;    //debug led
extern DigitalOut blue;    //debug led

#define BLUE_ON	1   // ericyang 20170103
#define	BLUE_OFF 0	// ericyang 20170103

#ifdef LED_INFO_ENABLE  // ericyang 20161226
#define WARNING_LED_ON 		(red = 1)
#define WARNING_LED_OFF 	(red = 0)
#define CHARGING_LED_ON 	(green = 1)
#define CHARGING_LED_OFF 	(green = 0)
#define CONNECT_LED_ON 		(blue = 1)
#define CONNECT_LED_OFF 	(blue = 0)
#define SERVER_CONNECT_LED_ON  (server = 1)   // ericyang 20170120
#define SERVER_CONNECT_LED_OFF (server = 0)		// ericyang 20170120
#endif

#define RELAY_ON	1   // ericyang 20170111
#define RELAY_OFF 0		// ericyang 20170111

#endif
