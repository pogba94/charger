/*
* Note:last modification by orange cai in 20170720
*/

#ifndef EVCHARGER_H
#define EVCHARGER_H
#include "UserConfig.h"

/* Modify in 20171221 */
typedef enum {
    RESP_OK = 100, 
	  RESP_PARAM_ERROR = 101,
	  RESP_TIMEOUT = 102,
    RESP_ILLEGAL = 103,
} RespCode;

typedef enum {
    invalidID = 0,
 //   setChargingTime,
    setChargingStart,   
    setChargingEnd,
	  setUpdateVersion, // add by orangeCai in 20170328 
	  setCalibrateTime, //20170720 
    getChargerStatus,
    getChargingInfo,
	  getCurVersion,    // add by orangeCai in 20170607
	  getDate,     // 20170720
    notifyNewDevice,
    notifyChargerStatus,
//    notifyConnectStatus,
    notifyChargingInfo,
    notifyEndCharging,
//    notifyCheckVersion,
//	  notifyUpdateSuccess,
    notifyOTAResult, //20170720 
	  notifyUpdateVersion,
    unknownMsgID
} MSGID;

typedef enum {
    invalidType,
    req,
    resp,
} MSGType;

typedef enum {
    offline = 0,
    connected = 1,
    charging = 2,
    booked =3,
	  idle = 4,
  //  fault = 4,
  //  meterError = 4001,
  //  deviceError = 4002
    errorCode = 0x4000,
} ChargerStatus;

//add in 20170719
typedef enum{
	END_OVER_ENERGY = 0,
	END_OVER_TIME = 1,
	END_FULL_CHARGING = 2,
	END_IN_ADVANCE = 10,
	END_IN_ADVANCE_FULL = 11,
	END_WAIT_TIMEOUT = 12,
	END_OFFLINE = 20,
	END_CONNECT_EXCEPTION = 21,
	END_METER_EXCEPTION = 22,
	END_CURRENT_ERROR = 23,
	END_VOLTAGE_ERROR = 24,
	END_NONE = -1,
}ChargingEndTypes;

typedef struct {
    char  meterNumber[9];  //"15500544" 
    float energy; // unit: KWH
    float voltage;  // unit: V
    float current;  // unit: A
    float power;    // unit: W
    int   setDuration; //total charging time set by Wechat UI
    int   duration; // current total charging time
    int   status; //0->offline 1->idle 2-> charging 3-> booked 4-> error and other
    int  	connect; //0-> disconnect 1-> connect but not enable charging 2-> connect and charging enabled
	  int  	userId;  //20170719
	  int  	chargingType;//20170719,type=0--> money,type=1--> time, type=2--> full
		float setEnergy; //20170719,unit:kwh,format:%0.1f
} ChargerInfo;

/*typedef struct {
    float energy; // unit: KWH
    float voltage;  // unit: V
    float current;  // unit: A
    float power;    // unit: W
} MeterInfo;
*/
typedef struct{
    bool serverConnectedFlag;  // device can't connect to server
    bool meterCrashedFlag;    // meter can't read out voltage/current/energy
    bool chargingVoltageErrorFlag;  // voltage: 176~264V
    bool chargingCurrentErrorFlag;  // current: 0 ~ 16A
    bool chargingTimeOverFlag; 
    bool chargingEnableFlag;  // Auto enable charging,  PWM 6V  detected 
	  bool chargingEnergyOverFlag; //20170719
} ChargerException;

typedef struct _OTAInfo {
    char latestVersionFromServer[VERSION_STR_LEN];
	  char latestVersionSNFromServer[VERSION_SN_LEN];   //orangecai 20170410
    int currentFWSector;  // 0~32
    int FWSectorNum;
    int lastSectorSize;
    int FWSizeFromServer;
    int FWCheckSumFromServer;
	  int oldVersionId;    //orangecai 20170414
    int newVersionId;    //orangecai 20170414	
} OTAInfo;

typedef enum _connectStatus{
    NOT_CONNECTED = 0,
    CONNECTED_9V = 1,
    CONNECTED_6V = 2,    
}connectStatus;

typedef enum _chargingTypes{
   CHARGING_TYPE_MONEY = 0,
	 CHARGING_TYPE_TIME = 1,
	 CHARGING_TYPE_FULL = 2,
	 CHARGING_TYPE_NONE = -1,
}chargingTypes;

/* added in 20171114*/
#ifdef NETWORK_COUNT_ENABLE
typedef struct _networkCount{
	uint32_t sendCnt;   //numbers of massages sended
	uint32_t recCnt;    //numbers of massages recieved
	uint32_t reSendCnt; //numbers of massages resended
}NetworkCount;
#endif

typedef struct _msg{
	bool sendRespFlag;
	MSGID msgId;
	int respCode;
	char msgIdStr[10];
}MSG;

#define CHARGER_STATUS_MASK            0x03   //2 LSBs
#ifndef CHARGING_EXCEPTION_HANDLE_ENABLE
#define METER_CRASHED_FLAG(x)              ((x) << 1)
#define CHARGING_VOLTAGE_ERROR_FLAG(x)     ((x) << 2)
#define CHARGING_CURRENT_ERROR_FLAG(x)     ((x) << 3)
#define CHARGING_TIME_OVER_FLAG(x)         ((x) << 4)
#define CHARGING_DISABLE_FLAG(x)           ((x) << 5)
#define CHARGING_ENERGY_OVER_FLAG(x)       ((x) << 6) //orangecai 20170719 
#endif

#ifdef CHARGING_EXCEPTION_HANDLE_ENABLE
#define METER_CRASHED_FLAG(x)              ((x) << 4)
#define CHARGING_VOLTAGE_ERROR_FLAG(x)     ((x) << 5)
#define CHARGING_CURRENT_ERROR_FLAG(x)     ((x) << 6)
#define CHARGING_DISABLE_FLAG(x)           ((x) << 7)
#define CHARGING_TIME_OVER_FLAG(x)         ((x) << 8)
#define CHARGING_ENERGY_OVER_FLAG(x)       ((x) << 9) //orangecai 20170719
#endif

#define SOCKET_OTA_HEADER  "OTABIN"

#ifdef  EEPROM_ENABLE
	#define  EEPROM_ADDRESS_WRITE      0xA0         
	#define  EEPROM_ADDRESS_READ       0xA1
	#define  EEPROM_PAGE_SIZE           16
#endif

extern ChargerInfo chargerInfo;
extern ChargerException chargerException;
extern OTAInfo otaInfo;
extern MSGID notifySendID;
extern int notifySendCounter;
extern int chargingEndType;

void parseRecvMsgInfo(char *text);
void parseBincodeBuffer(char *text);
void initCharger(void);
void notifyMsgSendHandle(MSGID msgid);
void cmdMsgRespHandle(MSGID msgid);
int startCharging(void);
int stopCharging(void);
void initailizeChargerInfo(void);
void chargingExceptionHandle(void);
int getChargingEndType(int chargerStatus);//orangecai 20170719
#if defined(EEPROM_ENABLE)
void saveChargerInfoToEEPROM(void);
void loadChargerInfoFromEEPROM(void);
uint8_t writeToEEPROM(uint8_t addr,char* data,uint8_t size);
void readFromEEPROM(uint8_t addr,char* data,uint8_t size);
#else
void loadChargerInfoFromFlash(void);
void saveChargerInfoToFlash(void);
#endif

#ifdef GB18487_1_2015_AUTH_FUNC
extern int volatile gChargingState;
extern volatile bool startS2SwitchOnOffCounterFlag;
extern volatile int S2SwitchOnOffCounter;
extern volatile bool pauseChargingFlag;
extern volatile int pauseChargingCounter;   //orangecai 20170608
int preStartCharging(void);
int preStopCharging(void);
int pauseCharging(bool onoff);
#endif

//add by orangeCai,20170525
#ifdef CHARGING_EXCEPTION_HANDLE_ENABLE
extern volatile bool chargingExceptionFlag;  //the flag would be set when exception happened while charging
extern volatile bool rechargingFlag;    // the flag would be set when exception disappeared while charging
extern volatile bool startContinueChargingFlag;
extern volatile uint32_t chargingErrorTimer;
extern volatile uint32_t rechargingTimer;
extern volatile uint8_t rechargingWaitCounter;
#endif
extern volatile bool updateCodeFlag;
extern volatile bool otaSuccessFlag;  //orangecai 20170720
extern volatile bool startS2SwitchWaitCounterFlag;  //orangecai 20170807
extern volatile int waitS2SwitchOnCounter; //orangecai 20170807
extern MSG respMsgBuff;  // orangecai 20171221
#ifdef RTC_ENABLE
extern char timeStampMs[32];
#endif
#ifdef NETWORK_COUNT_ENABLE
extern volatile NetworkCount networkCount;
#endif

//#define SETChargingTime         "setChargingTime"
#define SETChargingStart        "setChargingStart"  // ericyang 20160819
#define SETChargingEnd          "setChargingEnd"     // ericyang 20160819
#define SETUpdateVersion        "setUpdateVersion"  // orangeCai 20170328
#ifdef RTC_ENABLE
	#define SETCalibrateTime        "setCalibrateTime"  //orangecai 20170720
#endif
#define GETChargerStatus        "getChargerStatus"
#define GETChargingInfo         "getChargingInfo"
#define GETCurVersion           "getCurVersion"      //orangecai 20170607
#ifdef RTC_ENABLE
	#define GETDate                 "getDate"           //orangecai 20170720
#endif
#define NOTIFYNewDevice         "notifyNewDevice"
#define NOTIFYChargerStatus     "notifyChargerStatus"
//#define NOTIFYConnectStatus     "notifyConnectStatus"
#define NOTIFYChargingInfo      "notifyChargingInfo"
#define NOTIFYEndCharging       "notifyEndCharging"
//#define NOTIFYCheckVersion      "notifyCheckVersion"
#define NOTIFYUpdateVersion     "notifyUpdateVersion"
//#define NOTIFYUpdateSuccess     "notifyUpdateSuccess"   //orangeCai 20170328
#define NOTIFYOTAResult         "notifyOTAResult"       //orangecai 20170720
#define NOTIFYHeartPackage      "notifyHeartPackage"    //orangeCai 20170328

//#define CMD_RESP_setChargingTime    "{\"respType\":\"setChargingTime\",\"data\":{\"respCode\":%d,\"msgId\":\"%s\"}}"

#define CMD_RESP_setChargingStart   "{\"respType\":\"setChargingStart\",\"data\":{\"respCode\":%d,\"msgId\":\"%s\"}}"
#define CMD_RESP_setChargingEnd     "{\"respType\":\"setChargingEnd\",\"data\":{\"respCode\":%d,\"msgId\":\"%s\",\"type\":%d,\"energy\":%0.1f,\"duration\":%d,\"status\":%d,\"connect\":%d,\"setDuration\":%d,\"setEnergy\":%0.1f}}" //20170719
#define CMD_RESP_ILLEGAL_setChargingEnd  "{\"respType\":\"setChargingEnd\",\"data\":{\"respCode\":%d,\"msgId\":\"%s\"}}"  //20171222
#define CMD_RESP_setUpdateVersion   "{\"respType\":\"setUpdateVersion\",\"data\":{\"respCode\":%d,\"msgId\":\"%s\"}}"  //orangeCai 20170328
#ifdef RTC_ENABLE
	#define CMD_RESP_setCalibrateTime   "{\"respType\":\"setCalibrateTime\",\"data\":{\"respCode\":%d,\"msgId\":\"%s\"}}"//orangecai 20170720
#endif
#define CMD_RESP_getChargerStatus   "{\"respType\":\"getChargerStatus\",\"data\":{\"respCode\":%d,\"msgId\":\"%s\",\"status\":%d,\"connect\":%d}}"
#define CMD_RESP_getChargingInfo    "{\"respType\":\"getChargingInfo\",\"data\":{\"respCode\":%d,\"msgId\":\"%s\",\"energy\":%0.1f,\"voltage\":%0.2f,\"current\":%0.2f,\"power\":%0.2f,\"duration\":%d,\"status\":%d,\"connect\":%d,\"setDuration\":%d,\"setEnergy\":%0.1f}}"
#define CMD_RESP_getCurVersion      "{\"respType\":\"getCurVersion\",\"data\":{\"respCode\":%d,\"msgId\":\"%s\",\"versionNumber\":\"%s\"}}"  //orangecai 20170607,20170719
#ifdef RTC_ENABLE
	#define CMD_RESP_getDate            "{\"respType\":\"getDate\",\"data\":{\"respCode\":%d,\"msgId\":\"%s\",\"date\":\"%s\"}}"  //orangecai 20170720
#endif
#define NOTIFY_REQ_NewDevice        "{\"reqType\":\"notifyNewDevice\",\"data\":{\"mac\":\"%s\",\"isReconnect\":%d}}"
#define NOTIFY_REQ_ChargerStatus    "{\"reqType\":\"notifyChargerStatus\",\"data\":{\"status\":%d,\"connect\":%d}}"
//#define NOTIFY_REQ_ConnectStatus    "{\"reqType\":\"notifyConnectStatus\",\"data\":{\"connect\":%d}}"
#define NOTIFY_REQ_ChargingInfo     "{\"reqType\":\"notifyChargingInfo\",\"data\":{\"type\":%d,\"energy\":%0.1f,\"voltage\":%0.2f,\"current\":%0.3f,\"power\":%0.1f,\"duration\":%d,\"status\":%d,\"connect\":%d,\"setDuration\":%d,\"setEnergy\":%0.1f}}"//20170719
//#define NOTIFY_REQ_ChargingInfo     "{\"reqType\":\"notifyChargingInfo\",\"data\":{\"energy\":%0.1f,\"voltage\":%0.2f,\"current\":%0.3f,\"power\":%0.1f,\"duration\":%d,\"status\":%d,\"connect\":%d,\"setDuration\":%d}}"
#define NOTIFY_REQ_EndCharging      "{\"reqType\":\"notifyEndCharging\",\"data\":{\"userId\":%d,\"energy\":%0.1f,\"duration\":%d,\"status\":%d,\"connect\":%d,\"setDuration\":%d,\"setEnergy\":%0.1f,\"endType\":%d}}"//20170719
//#define NOTIFY_REQ_CheckVersion     "{\"reqType\":\"notifyCheckVersion\"}"

#define NOTIFY_REQ_UpdateVersion    "{\"reqType\":\"notifyUpdateVersion\",\"data\":{\"versionSN\":\"%s\",\"blockOffset\":%d,\"blockSize\":%d}}" //orangecai 20170410

//#define NOTIFY_REQ_UpdateSuccess    "{\"reqType\":\"notifyUpdateSuccess\",\"data\":{\"oldVersionId\":%d,\"newVersionId\":%d}}" //orangecai 20170414
#define NOTIFY_REQ_OTAResult        "{\"reqType\":\"notifyOTAResult\",\"data\":{\"result\":%d,\"oldVersionId\":%d,\"newVersionId\":%d}}" //orangecai 20170720
#define NOTIFY_REQ_HeartPackage     "{\"reqType\":\"notifyHeartPackage\",\"data\":{\"ledStatus\":%d}}" //orangeCai 20170328

#endif
