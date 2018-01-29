/*------------------------------------------------------------------------------------*/
/*  Ethernet EV Charger Solution                          */
/*------------------------------------------------------------------------------------*/

/*--COMPANY-----AUTHOR------DATE------------REVISION----NOTES-------------------------*/
/*  WPI        Eric.Yang  2016.07.03       rev 1.0     initial                       */
/*  Function:
				mbed
        Ethernet TCP Client
        JSON
        Modbus(RS485)
        CRC16
        LCD12832
        EV Charger (Wechat websocket)
				OTA & IAP			
				CP Signal Handle
*/

/*--INCLUDES----------------------------------------------------------------------------*/
#include "UserConfig.h"
#include "EVCharger.h"
#include "cJSON.h"
#include "lib_crc16.h"
#include "flashLayout.h"
#include "otaCharger.h"

#ifdef LCD_ENABLE
#include "C12832.h"
#include "Arial12x12.h"
#include "Small_7.h"
C12832 lcd(D11, D13, D12, D7, D10);
#endif

#ifdef WDOG_ENABLE
#include "WatchDog.h"
#endif

#ifdef EEPROM_ENABLE
extern "C"{
	#include "MK64F12.h"
}
#endif
/*--DEFINES-----------------------------------------------------------------------------*/
#define RESET_TIMER 120

//#define BOARD_FRDM_K64F            //use board frdm_k64f for test!

//#define ABS(x) ((x)<0 ? -(x) :x)   // ericyang 20161209
//#define ADC_SAMPLE_TIMES 2  // ericyang 20161214 for CP TEST < 100MS
/*--CONSTANTS---------------------------------------------------------------------------*/

const char* DEFAULT_SERVER_ADDRESS = "112.74.170.197";

const int DEFAULT_SERVER_PORT = 22221;


char ECHO_SERVER_ADDRESS[20];
int ECHO_SERVER_PORT;
//const char* DEFAULT_SERVER_ADDRESS = "192.168.1.100";
//const int DEFAULT_SERVER_PORT = 13348;//13348;


/*--INITS-------------------------------------------------------------------------------*/
Serial pc(USBTX, USBRX);      //create PC interface
//DigitalOut switchRelayOnoff(D2);//PTB9
DigitalOut switchRelayOnoff(PTB23);//PTB23
//AnalogIn analog0(PTB0);
//AnalogIn analog1(PTB1);
AnalogIn cpADC(A0);  // adc for cp detect PTB2
//AnalogIn cpADC(PTB3);
//AnalogIn analog4(PTC2);
//AnalogIn cpADC(PTC1);
//PwmOut cpPWM(A5);  // cp pwm signal generator PTC10
PwmOut cpPWM(D6);  // cp pwm signal generator PTC2
//Serial RS485(D1,D0); // Tx, Rx
//Serial RS485(PTC4,PTC3); // Tx, Rx D9/D7
//Serial RS485(PTD3,PTD2);  // D12/D11
Serial RS485(PTE24,PTE25);  // D15/D14
EthernetInterface eth;          //create ethernet
TCPSocketConnection tcpsocket;

#if defined(BOARD_FRDM_K64F)
	DigitalOut red(PTB10);        //warning led
	DigitalOut green(PTB11);    //charging led
	DigitalOut blue(PTC10);    //connect led
	DigitalOut server(PTC11);    //server connected led
#else
	DigitalOut red(LED_RED);        //warning led
	DigitalOut green(LED_GREEN);    //charging led
	DigitalOut blue(LED_BLUE);    //connect led
	DigitalOut server(PTB20);    //server connected led
#endif

InterruptIn sw2(SW2);

DigitalIn factoryMode(PTD7);  //

#ifdef EEPROM_ENABLE
I2C i2c1(PTC11,PTC10);    //orangecai I2C1 Read Epprom data
#endif
//InterruptIn sw3(SW3);

Mutex myMutex;
char EVChargerModelNO[20]; // 20160713 ericyang set MAC as EVCharger Model NO
SystemEventHandle eventHandle;
SocketInfo socketInfo;
int systemTimer = 0;
int resetTimer = 0;
float startEnergyReadFromMeter = 0;
float totalEnergyReadFromMeter = 0;
bool pwmState = false;
MSG respMsgBuff = {false,invalidID,0,{0}};  // orangecai 20171221

volatile bool pauseChargingFlag = false;
volatile bool startS2SwitchWaitCounterFlag = false;   //orangecai 20170807
volatile int pauseChargingCounter = 0;   //orangecai 20170608
volatile int waitS2SwitchOnCounter = 0; //orangecai 20170807
#ifdef GB18487_1_2015_AUTH_FUNC
	volatile bool S2SwitchEnabledFlag = false; // ericyang 20170711
	volatile bool startS2SwitchOnOffCounterFlag = false;
	volatile int S2SwitchOnOffCounter = 0;
	volatile int gChargingState = EV_IDLE;
#endif

#ifdef AUTOTEST_SOCKET_JSON_MSG
	bool testModeFlag = false;
#endif

char tempBuffer[256];//need to be fixed, save data flash use this buffer;
char dataBuffer[256];
#ifdef RTC_ENABLE
	char timeStampMs[32];
#endif

#if 0
	typedef struct {
			bool ethernetInitedFlag;  // ethernet init ok=true;
			bool ethernetConnectedFlag; // network is not ready, RJ45 no insert in or other problem
	} EthernetException;
	EthernetException ethernetException;
#endif

#ifdef NETWORK_COUNT_ENABLE
volatile NetworkCount networkCount = {0,0,0}; 	
#endif	
	
//add by orangeCai,20170525
#ifdef CHARGING_EXCEPTION_HANDLE_ENABLE
	volatile bool chargingExceptionFlag = false;   //the flag would be set when exception happened while charging
	volatile bool rechargingFlag = false;    // the flag would be set when exception disappeared while charging
	volatile bool startContinueChargingFlag = false; 
	volatile uint32_t chargingErrorTimer = 0;
	volatile uint32_t rechargingTimer = 0;
	volatile uint8_t rechargingWaitCounter = 0;
#endif
	volatile bool updateCodeFlag = false;   //Start to update code when the flag is set and the status is not charging,orangeCai 20170602
  volatile bool otaSuccessFlag = false;   //true:OTA code download success
#ifdef ENABLE_FULL_CHARGING_DETECT
	volatile uint16_t fullChargingDetectCounter = 0;
	volatile bool fullChargingFlag = false;
#endif

/*--FUNCTION DECLARATIONS---------------------------------------------------------------*/
void initServer(void); // 20161107 ADD FOR SERVER IP COULD BE CONFIGED
void initETH(void);    //initializes Ethernet
void enterFactoryMode(void);
void timerOneSecondThread(void const *argument);
void heartbeatThread(void const *argument);
#ifdef LCD_ENABLE
	void lcdshowonoffstate(bool onoffstate);
#endif
void checkNetworkAvailable(void);
void checkChargingTime(void);
//void getChargerInfo(void);
void checkChargerConnectStatus(void);
int getMeterInfo(void);
void sw2_release(void);

//void disableCPPWM(void);
//void enableCPPWM(float duty);
CPSignal checkCPSignal(void);

/*--FUNCTION DEFINITIONS----------------------------------------------------------------*/

#ifdef EEPROM_ENABLE
static void pinConfig(void)
{
	/* config I2C1 SCL pin , PTC10 */
	PORTC->PCR[10] |= (uint32_t)0x03; 
	/* config I2C1 SDA pin , PTC11 */
	PORTC->PCR[11] |= (uint32_t)0x03;
}
#endif

#ifdef RTC_ENABLE
void getTimeStampMs(char* timeStampMs)
{
  time_t seconds;
	char buffer[32];
	seconds = rtc_read();
	strftime(buffer, 32, "%y%m%d%H%M%S", localtime(&seconds));
	sprintf(timeStampMs,"%s%d",buffer,us_ticker_read()%1000000/1000);
}

void rtcInit(uint32_t timeSeconds)
{
	char* initTime;
	set_time(timeSeconds);  //time in seconds since January 1, 1970
	initTime = ctime(&timeSeconds);
	pc.printf("init time:%s\r\n",initTime);
	us_ticker_init(); 
}

#endif

void sw2_release(void)
{
	#ifndef GB18487_1_2015_AUTH_FUNC
    blue = !blue;
	#endif
	//	switchRelayOnoff = !switchRelayOnoff;
}

/*****************************************INIT_ETH***********************************************/
void initETH(void)
{
    uint8_t MAC_ADDRESS[6];   // 20160713 ericyang set MAC as EVCharger Model NO
//    eth.init("192.168.1.101", "255.255.255.0", "192.168.1.1");                                         //set up static IP
//    pc.printf("eth.init: %d\r\n",eth.init());
    eth.init(); //DHCP
    eth.connect();                                                              //connect ethernet
    pc.printf("MAC Addr:%s\r\nIP Addr:%s\r\n",eth.getMACAddress(),eth.getIPAddress());    //get client IP address
    if(strcmp(eth.getIPAddress(),NULL) == NULL) {
        pc.printf("RJ45 error! system will be reset now, please wait...\r\n");
        NVIC_SystemReset();
    }
    sscanf(eth.getMACAddress(),"%02x:%02x:%02x:%02x:%02x:%02x",&MAC_ADDRESS[0],&MAC_ADDRESS[1],&MAC_ADDRESS[2],&MAC_ADDRESS[3],&MAC_ADDRESS[4],&MAC_ADDRESS[5]);
    sprintf(EVChargerModelNO,"%02x%02x%02x%02x%02x%02x",MAC_ADDRESS[0],MAC_ADDRESS[1],MAC_ADDRESS[2],MAC_ADDRESS[3],MAC_ADDRESS[4],MAC_ADDRESS[5]);
//    pc.printf("%s\r\n", EVChargerModelNO);
}

/*****************************************init Server IP ADDRESS****************************************/
void initServer(void)
{
    char* cdata = (char*)SERVER_IP_ADDRESS;
    uint16_t crc16,tempcrc16 = 0;
    crc16 = calculate_crc16(cdata, 6);
    tempcrc16 = (cdata[6] << 8) | cdata[7];
    if(crc16 == tempcrc16) {
        sprintf(ECHO_SERVER_ADDRESS,"%d.%d.%d.%d",cdata[0],cdata[1],cdata[2],cdata[3]);
        ECHO_SERVER_PORT = (cdata[4]<<8)|cdata[5];
    } else {
        sprintf(ECHO_SERVER_ADDRESS,"%s",DEFAULT_SERVER_ADDRESS);
        ECHO_SERVER_PORT = DEFAULT_SERVER_PORT;
    }
    pc.printf("initServer: %s:%d\r\n",ECHO_SERVER_ADDRESS,ECHO_SERVER_PORT);
}
/*****************************************initEventHandle***********************************************/
void initEventHandle(void)
{
    eventHandle.heatbeatFlag = false;
    eventHandle.checkMeterFlag = false;
    eventHandle.updateChargerInfoFlag = false;
    eventHandle.updateChargerStatusFlag = false;  // ericyang 20160824
//    eventHandle.updateConnectStatusFlag = false;
    eventHandle.stopChargingFlag = false;
//    eventHandle.checkLatestVersionFromServerFlag = false; // ericyang 20160914
    eventHandle.getLatestFWFromServerFlag = false; // ericyang 20160914
	  eventHandle.updateVersionDoneFlag = false;  //orangeCai 20170328
	  eventHandle.firstConnectFlag = true;   //orange 20171213
}
/***********************************1s timer thread***************************/
void timerOneSecondThread(void const *argument)
{
    while (true) {	
			if(systemTimer >= 3){
						//if(systemTimer % 60 == 0)
						eventHandle.checkMeterFlag = true; // 1s check meter info
        }
				
        if(chargerException.serverConnectedFlag == false) {
            resetTimer++;
//					  pc.printf("resetTimer=%d\r\n",resetTimer);
        }
#ifdef RTC_ENABLE
		getTimeStampMs(timeStampMs);
		pc.printf("time:%s\r\n",timeStampMs);
#endif
#ifdef ENABLE_FULL_CHARGING_DETECT
				if(chargerInfo.status == charging){
					if(chargerInfo.current > 0.9f && chargerInfo.current < 1.1f){
						fullChargingDetectCounter++;
					}else{
						fullChargingDetectCounter = 0;
					}
					if(fullChargingDetectCounter >= FULL_CHARGING_DETECT_TIME){
						fullChargingFlag = true;
						fullChargingDetectCounter = 0;
					}
				}else{
					fullChargingDetectCounter = 0;
					fullChargingFlag = false;
				}
#endif

#ifdef CHARGING_EXCEPTION_HANDLE_ENABLE
				if(rechargingFlag == true){
						if(rechargingTimer > NORMAL_TIME_HOLDED){
							  startContinueChargingFlag = true;  // replace with preStartCharging()
							//	preStartCharging();   // continue charging
								rechargingFlag = false;
								rechargingWaitCounter = 0;
								rechargingTimer = 0;
						}else if(chargerInfo.status != connected){
								if(++rechargingWaitCounter > RECHARGING_WAIT_TIMES_ALLOWED){
									  rechargingWaitCounter = 0;
										eventHandle.stopChargingFlag = true; //notify END charging msg to server
									  chargingEndType = getChargingEndType(chargerInfo.status);//orangecai 20170719
										initailizeChargerInfo();
                    pc.printf("\r\nrecharging fail ! stop charging!\r\n\r\n");									
								}else{
										chargingExceptionFlag = true;
								}
								rechargingFlag = false;
								rechargingTimer = 0;							
								pc.printf("\r\nrechargingWaitCounter =%d\r\n\r\n",rechargingWaitCounter);
						}else{
								rechargingTimer++;
							  pc.printf("rechargingTimer = %d \r\n",rechargingTimer);
						}
				}

				if(chargingExceptionFlag  == true){
					if(chargingErrorTimer > MAX_EXCEPTION_TIME_ALLOWED){
							chargingExceptionFlag = false;
						  chargingErrorTimer = 0;  //clear timer
						  rechargingWaitCounter = 0;
						  eventHandle.stopChargingFlag = true;  //notify END charging msg to server
						  chargerInfo.status &= 0xFFFC;
						  chargingEndType = getChargingEndType(chargerInfo.status);//orangecai 20170719
						  initailizeChargerInfo();
              pc.printf("Exception Timeout,stop charging!\r\n");						
					}else if(chargerInfo.status == connected){
							rechargingFlag = true;
						  chargingExceptionFlag = false;
						  chargingErrorTimer = 0;
						  rechargingTimer = 0;
						  pc.printf("rechargingFlag = true,prepare to continute charging!\r\n");
					}else
						chargingErrorTimer++;
				}
#endif
#ifdef AUTOTEST_SOCKET_JSON_MSG
        if(systemTimer % 300 == 0)
            testModeFlag = true;
#endif
#ifdef GB18487_1_2015_AUTH_FUNC
				if(startS2SwitchOnOffCounterFlag == true)
						S2SwitchOnOffCounter++;
#endif
        Thread::wait(1000);
        systemTimer++;
        if(chargerInfo.status == charging) // ericyang 20170111 modify
          chargerInfo.duration++;
				if(pauseChargingFlag == true){   //orangecai 20170608
					pauseChargingCounter++;
				}
				if(startS2SwitchWaitCounterFlag == true){
					waitS2SwitchOnCounter++;
				}
    }
} 

/***********************************30s timer thread***************************/
void heartbeatThread(void const *argument)
{
#ifdef DISABLE_NETWORK_CONMUNICATION_FUNC
    static int recoverTimer = 0;
#endif
    while (true) {
        Thread::wait(30000);   
#ifdef DISABLE_NETWORK_CONMUNICATION_FUNC
        if(eventHandle.stopCommunicationFlag == true) {
            if(recoverTimer > 1) {
                eventHandle.stopCommunicationFlag = false;
                pc.printf("start Conmunication!\r\n");
                recoverTimer = 0;
            } else
                recoverTimer++;
        } else
#endif
        {
            eventHandle.heatbeatFlag = true;
        }
    }
}

/***********************************display 12832 server connect info  ***************************/
#ifdef LCD_ENABLE
void lcdshowonoffstate(bool onoffstate)
{
    myMutex.lock();
    lcd.locate(0,21);
    if(onoffstate == false) {
        red = 0;
        green = 1;
        pc.printf("disconnected!\r\n");
        lcd.printf("IP: %s Offline!",eth.getIPAddress());
    } else {
        red = 1;
        green = 0;
        pc.printf("connected!\r\n");
        lcd.printf("IP: %s Online!",eth.getIPAddress());
    }
    myMutex.unlock();
}
#endif

/***********************************update meter info Modbus(RS485)***************************/
#ifdef TEST_NO_485
int getMeterInfo(void)
{
		chargerException.meterCrashedFlag = false; 
		chargerException.chargingCurrentErrorFlag = false;
		chargerException.chargingVoltageErrorFlag = false;
		return 0;
}
#else
int getMeterInfo(void)
{
    char ModbusData[8] = {0x2C,0x03,0x00,0x00,0x00,0x07};// slave addr,function code,Hi PDU addr,low PDU addr,Hi N reg,Lo N reg,Lo CRC,Hi CRC
    char regvalue[20] = {0};
    uint16_t crc16;
    int i,count;
    static int meterErrorTimes = 0;
		static int voltageErrorTimes = 0;
		static int currentErrorTimes = 0;
		
//    static int lowCurrentTimes = 0;
    ModbusData[0] = atoi(chargerInfo.meterNumber + 6); // get last two number [6][7] as meter addr
    pc.printf("meter address: %x\r\n",ModbusData[0]);

    crc16 = calculate_crc16_Modbus(ModbusData, 6);
    //pc.printf("crc  : %04x\r\n",crc16);
    ModbusData[6] = crc16 & 0xff;
    ModbusData[7] = (crc16 >> 8) & 0xff;

    //  ////myMutex.lock();
    for(i = 0; i < 8; i++)
        RS485.putc(ModbusData[i]);

    i = 0;
    count = 0;

    while((i < 19) && (count < 50)) {

        if(RS485.readable()) {
            count = 0;
            regvalue[i++] = RS485.getc(); // Retrieving received register from modbus
        } else {
					  wait_ms(1);
            count++;
        }
    }

    //  ////myMutex.unlock();
    crc16 = calculate_crc16_Modbus(regvalue,17);
//		for(int i =0;i<19;i++)
//				pc.printf("reg[%d] = %x\t",i,regvalue[i]);
    //pc.printf("receiver crc  : %04x\r\n",crc16);
    if(( (crc16 & 0xff) == regvalue[17]) && (((crc16 >> 8) & 0xff) == regvalue[18])) {
        //      pc.printf("Modbus read successful!\r\n");
        meterErrorTimes = 0;
        if(chargerException.meterCrashedFlag == true) {
            //    eventHandle.updateChargerStatusFlag = true; // ericyang 20160824
            chargerException.meterCrashedFlag = false; // ericyang 20160824 meter recover ok
        }

        totalEnergyReadFromMeter = (regvalue[5]<<24 | regvalue[6]<<16 | regvalue[3]<<8 | regvalue[4])*0.1;
        chargerInfo.energy =  totalEnergyReadFromMeter - startEnergyReadFromMeter;
//				chargerInfo.energy += 0.02;//for test!
        chargerInfo.voltage = (regvalue[7]<<8 | regvalue[8])*0.01;
        chargerInfo.current = (regvalue[11]<<24 | regvalue[12]<<16 | regvalue[9]<<8 | regvalue[10])*0.001;
        chargerInfo.power = ((regvalue[15]&0x7F)<<24 | regvalue[16]<<16 | regvalue[13]<<8 | regvalue[14])*0.1;
        //eventHandle.updateChargerInfoFlag = true; // ericyang 20161215 move to 1 second thread
        if((chargerInfo.voltage > 264) || (chargerInfo.voltage < 176)) {  // voltage: 176~264
						if((chargerException.chargingVoltageErrorFlag == false)&&(voltageErrorTimes++ >= 5)) {
                chargerException.chargingVoltageErrorFlag = true;
            }
            pc.printf("\r\n Voltage Error!!!\r\n"); 
        } else {
						voltageErrorTimes = 0;
            chargerException.chargingVoltageErrorFlag = false;
        }
        if(chargerInfo.status == charging) {
            //if((chargerInfo.current > 17) ||(chargerInfo.current < 0.01)){ // current: 0~20 0.1 just for test
					#ifdef CHARGING_CURRENT_16A
						if(chargerInfo.current > 15) // current: 0~18 13+2 >5s  fix 18487.1-2015 A3.10.7
					#endif
					#ifdef CHARGING_CURRENT_32A
						if(chargerInfo.current > 35.2)// current > 32*1.1=35.2A && >5s fix fix 18487.1-2015 A3.10.7
          #endif
            {						
							if((chargerException.chargingCurrentErrorFlag == false)&&(currentErrorTimes++ >= 4)){//max 5s
                chargerException.chargingCurrentErrorFlag = true;
							}
							pc.printf("\r\n Current out of range  Error!!!\r\n"); 
            }
						else {
								currentErrorTimes = 0;
                chargerException.chargingCurrentErrorFlag = false;
            }
        } else {
            if(chargerInfo.current > 0.2) { // leak current
							if((chargerException.chargingCurrentErrorFlag == false)&&(currentErrorTimes++ >= 5)){
                chargerException.chargingCurrentErrorFlag = true;
							}
							pc.printf("\r\n Current leakage  Error!!!\r\n"); 
            } else {
								currentErrorTimes = 0;
                chargerException.chargingCurrentErrorFlag = false;
            }
        }
				if((systemTimer % 30 == 0)&&(chargerInfo.status == charging)) { //30s refresh charger info  ericyang20161216 move here              
						eventHandle.updateChargerInfoFlag = true;
        }
        pc.printf("%0.1f - %0.2f - %0.3f - %0.1f \r\n",chargerInfo.energy,chargerInfo.voltage,chargerInfo.current,chargerInfo.power);  //voltage V/ current A/ power W
    } else { //default value
        if(chargerException.meterCrashedFlag == false) {
            if(meterErrorTimes++ >= 4) {
                chargerException.meterCrashedFlag = true;
                //eventHandle.updateChargerStatusFlag = true; // ericyang 20160824
                //chargerInfo.status = meterError; // ericyang 20160824
            }
            pc.printf("\r\nMeter Crashed!!!\r\n");
        }
        return -1; // read meter error!!!
    }

#ifdef LCD_ENABLE
    lcd.locate(0,1);
    lcd.printf("EE:%0.1fKWH Vol:%0.2fV Cur:%0.3fA",chargerInfo.energy,chargerInfo.voltage,chargerInfo.current);
    lcd.locate(0,11);
    //lcd.printf("Charging Time: %d min",chargerInfo.duration);
    lcd.printf("Charging Time: %02d : %02d : %02d",chargerInfo.duration/3600,(chargerInfo.duration/60)%60,chargerInfo.duration%60);
#endif
    return 0;
}
#endif

/******************************************initPWM****************************************/
void disableCPPWM(void)
{
    cpPWM.period_ms(0);
    cpPWM = 1;
    pwmState = false;
}

void enableCPPWM(float duty)
{
    cpPWM.period_ms(1);
    cpPWM = duty;
    pwmState = true;
}



int getMaxNo(float *array,int len)
{
    int i,pos=0;
    for(i=1;i<len;i++)
    {
        if(array[i]>=array[pos])
                pos = i ;
        
    }
    return pos;
}
int getMinNo(float *array,int len)
{
    int i,pos=0;
    for(i=1;i<len;i++)
    {
        if(array[i]<=array[pos])
                pos = i ;       
    }
    return pos;
}

#define ADC_SAMPLE_RATE 20     // 1ms 20 times
#define ADC_SAMPLE_USE_TOP_NUM  3
CPSignal checkCPSignal(void)
{
    float cpADCRead;
    float sumValue=0,averValue=0;
    float cp[20];
    int validCPNum=0;
		int minPos,maxPos;
		float top3Num[3];
    CPSignal ret = PWMNone;
    int i = 0;
	
    for(i=0; i<ADC_SAMPLE_RATE; i++) {
        cpADCRead = cpADC*3.3;
        //pc.printf("CP =%f\r\n",cpADCRead);
        cp[i] = cpADCRead;
        wait_us(50);//50*20 us = 1 ms
    }
		#if 0
		for(i=0; i<ADC_SAMPLE_RATE; i++) 
				pc.printf("cp[%d] = %f\r\n",i,cp[i]);
		#endif
		for(i=0;i<ADC_SAMPLE_USE_TOP_NUM;i++){  //ericyang 20170216 FIX_GB18487_TEST_PROBLEM
				maxPos = getMaxNo(cp,20);
				top3Num[i] = cp[maxPos];
				cp[maxPos] = 0;	 
		}
		#if 0
		for(i=0; i<ADC_SAMPLE_USE_TOP_NUM; i++) 
			 pc.printf("top3Num[%d]=%f\r\n",i,top3Num[i]);
		#endif	
		minPos = getMinNo(top3Num,ADC_SAMPLE_USE_TOP_NUM);
		maxPos = getMaxNo(top3Num,ADC_SAMPLE_USE_TOP_NUM);
		if(top3Num[maxPos] - top3Num[minPos] > 0.2)
				return ret;
		
		for(i=0; i<ADC_SAMPLE_USE_TOP_NUM; i++) {
			 sumValue += top3Num[i];
		}		
		averValue = sumValue / ADC_SAMPLE_USE_TOP_NUM;
		//if((averValue > 2.06)&&(averValue < 2.46)) { // (12-0.7) / 5  = 2.26
		if((averValue > 2.10)&&(averValue < 2.42)) { // (12-0.7) / 5  = 2.26  +/- (0.8/5) ericyang 20170216
				ret = PWM12V;
		//} else if((averValue > 1.46)&&(averValue < 1.86)) { // (9-0.7) / 5  = 1.66
			} else if((averValue > 1.50)&&(averValue < 1.82)) { // (9-0.7) / 5  = 1.66 +/- (0.8/5) ericyang 20170216
				ret = PWM9V;
		//} else if((averValue > 0.86) && (averValue < 1.26)) { // (6-0.7) / 5  = 1.06
			}	else if((averValue > 0.90) && (averValue < 1.22)) { // (6-0.7) / 5  = 1.06 +/- (0.8/5) ericyang 20170216
				ret = PWM6V;
		}	else if(averValue < 0.1) {
				ret = PWM0V;
		} else {
				ret = PWMOtherVoltage;
		}
		//pc.printf("checkCPSignal: result: %d  %f v\r\n",ret,averValue);
    return ret;
}
/***********************************check charger connect to car***************************/
void checkChargerConnectStatus(void)
{
		static int connectStatus = NOT_CONNECTED;  // 20170216 fixed
#ifdef GB18487_1_2015_AUTH_FUNC
    static CPSignal cpSignal = PWMNone;
		CPSignal tmpCPSignal = checkCPSignal();
		
		//if(tmpCPSignal != PWMNone)
		{
			if(cpSignal != tmpCPSignal)
			{
				cpSignal = tmpCPSignal;
				pc.printf("1: 12V  2: 9V 3: 6V 4: 0V 5: Other cpSignal=%d\r\n",cpSignal);
			}
		}
		//pc.printf("cpSignal=%d ,checkTimes =%d\r\n",cpSignal,checkTimes);
		//if((cpSignal == PWM12V)||(cpSignal == PWMNone)) {
		if((cpSignal == PWM12V)||(cpSignal == PWM0V)||(cpSignal == PWMOtherVoltage)) {
				connectStatus = NOT_CONNECTED;
				chargerException.chargingEnableFlag = false;
				S2SwitchEnabledFlag = false;  
		}
		else if((cpSignal == PWM9V) || (cpSignal == PWM6V)){
				if(cpSignal == PWM9V) {
						connectStatus = CONNECTED_9V;
						S2SwitchEnabledFlag = true;  // EV CAR S2 SWITCH DETECTED
				}
				else if(cpSignal == PWM6V) {
						connectStatus = CONNECTED_6V;
				}
				chargerException.chargingEnableFlag = true;
				#ifdef AUTO_CHARGING // ericyang 20170111
				if((chargerInfo.status == connected)&&(gChargingState == EV_IDLE)) {// ericyang 20170117
						pc.printf("Auto pre start charging!\r\n");
						chargerInfo.setDuration = 600;// 10 hours for test
						preStartCharging();
				}
				#endif
		}
		#ifdef LED_INFO_ENABLE		
		if(chargerException.chargingEnableFlag == true){
				CONNECT_LED_ON;
		}else{
				CONNECT_LED_OFF;
		}
		#endif		
#else
    if(blue == BLUE_ON) {  // CONNECT_LED_ON  ericyang 20161228
        connectStatus = CONNECTED_6V;
				//chargerInfo.status = idle;
			  chargerException.chargingEnableFlag = true;
				if(pwmState == false) {     // 16A current ericyang 20161230
					 #ifdef CHARGING_CURRENT_32A
						enableCPPWM(PWM_DUTY_CURRENT_32A);  // 32A current
					  pc.printf("sw2 =  1, enable PWM! PWM duty:%f\r\n",PWM_DUTY_CURRENT_32A);
					 #endif
					 #ifdef CHARGING_CURRENT_16A
					  enableCPPWM(PWM_DUTY_CURRENT_16A); //16A current
					  pc.printf("sw2 =  1, enable PWM! PWM duty:%f\r\n",PWM_DUTY_CURRENT_16A);
					 #endif
        }  
#ifdef AUTO_CHARGING // ericyang 20161208
        if(chargerInfo.status == connected) {
            pc.printf("sw2 button, start charging!\r\n");
            chargerInfo.setDuration = 600;// 10 hours for test
            startCharging();
        }
#endif
    } else {
        connectStatus = NOT_CONNECTED;
				chargerException.chargingEnableFlag = false;
				if(pwmState == true) { 
					disableCPPWM();// ericyang 20161230
					pc.printf("sw2 =  0, disable PWM!\r\n");
				}
    }
#endif
    if(connectStatus != chargerInfo.connect) {
        chargerInfo.connect = connectStatus; // ericyang 20160919
				pc.printf("chargerInfo.connect = %d\r\n",chargerInfo.connect);
        eventHandle.updateChargerStatusFlag = true;// ericyang 20161011 instead of updateConnectStatusFlag;
			  #if defined(EEPROM_ENABLE)
			  saveChargerInfoToEEPROM();
		    #else
        saveChargerInfoToFlash();
			  #endif
    }
}
/***********************************checkChargingStatus***************************/
#ifdef GB18487_1_2015_AUTH_FUNC
void checkChargingStatus(void)
{
		switch(gChargingState){
			case EV_CONNECTED_PRE_START_CHARGING:
					if(chargerInfo.connect == CONNECTED_6V){
							gChargingState = EV_CONNECTED_ON_CHARGING;
						#ifdef LED_INFO_ENABLE  // ericyang 20170216 add here
							CHARGING_LED_ON;
						#endif	
							if(chargerInfo.status == connected) {
									startCharging();   //Begin charging!
								  respMsgBuff.respCode = RESP_OK;
								  cmdMsgRespHandle(respMsgBuff.msgId);  //send respcond msg to server
							}
							else{
									if(pauseChargingFlag == true){  // recover charging ;do not notify server
											pauseCharging(false);
									}
							}
					}
					else{
						if(chargerInfo.connect != CONNECTED_9V){
								if(pauseChargingFlag == true){
										stopCharging();   //gChargingState = EV_IDLE;
										pauseChargingFlag = false;
								}
								disableCPPWM();// ericyang 20161230  GB/T 18487.1-2015 A3.10.9
								pc.printf("error connect, disable charging!\r\n");
//								#ifndef CHARGING_EXCEPTION_HANDLE_ENABLE
//								eventHandle.stopChargingFlag = true;  // notify END charging msg to server;		20170113 important notify stop charging msg to server	
//								#endif
								gChargingState = EV_IDLE;
								respMsgBuff.respCode = RESP_ILLEGAL; 
								cmdMsgRespHandle(respMsgBuff.msgId);  //send respcond msg to server
						}else{ //orangecai 20170608
								if(pauseChargingFlag == true && pauseChargingCounter > MAX_PAUSE_TIME){ //overtime of pause charging
										pauseChargingFlag = false;
									  eventHandle.stopChargingFlag = true;   // notify end charging to server
									  chargingEndType = END_IN_ADVANCE_FULL; //orangecai 20170719
										pauseChargingCounter = 0;
										stopCharging();   //gChargingState = EV_IDLE
									  disableCPPWM();   // fix bug 20170926
										#ifdef CHARGING_EXCEPTION_HANDLE_ENABLE
										initailizeChargerInfo();
                    chargerInfo.status = connected;									
										#endif
									  gChargingState = EV_IDLE;
								}
								if(startS2SwitchWaitCounterFlag == true && waitS2SwitchOnCounter > MAX_WAIT_TIME){
										startS2SwitchWaitCounterFlag = false;
										waitS2SwitchOnCounter = 0;
//										eventHandle.stopChargingFlag = true;
//										chargingEndType = END_WAIT_TIMEOUT;
//										stopCharging();
									  disableCPPWM();
										#ifdef CHARGING_EXCEPTION_HANDLE_ENABLE
										initailizeChargerInfo();
										chargerInfo.status = connected;
										#endif
									  gChargingState = EV_IDLE;
									  respMsgBuff.respCode = RESP_TIMEOUT;
									  cmdMsgRespHandle(respMsgBuff.msgId);  //send respcond msg to server
								}
						}
					}
					break;
			case EV_CONNECTED_ON_CHARGING:  // ericyang 20170113 fix: 18487.1-2015 A3.10.4 & A3.10.8
				  if(startS2SwitchWaitCounterFlag == true){
						startS2SwitchWaitCounterFlag = false;
						waitS2SwitchOnCounter = 0;
					}
					if(chargerInfo.connect != CONNECTED_6V){
						#ifdef LED_INFO_ENABLE  // ericyang 2017.02.16
							CHARGING_LED_OFF;
						#endif	
							if(chargerInfo.connect == CONNECTED_9V){  // S2 switch open  PWM still enable
									gChargingState = EV_CONNECTED_PRE_START_CHARGING;
									pauseCharging(true); // ericyang 20170113 add 
							}
							else{
									stopCharging();	  //gChargingState = EV_IDLE;
									disableCPPWM();
								#ifndef CHARGING_EXCEPTION_HANDLE_ENABLE
									eventHandle.stopChargingFlag = true;  // notify END charging msg to server;
                #endif								
									startS2SwitchOnOffCounterFlag = false;					
							}
					}
					break;
			case EV_CONNECTED_PRE_STOP_CHARGING:		// fix: 18487.1-2015 A3.9.2
			    if((S2SwitchEnabledFlag == true) && (chargerInfo.connect == CONNECTED_6V)) {
							if((startS2SwitchOnOffCounterFlag == true) && (S2SwitchOnOffCounter < 3)){   // 3s timeout , STOP charging!!!
									return;
							}	
					}		
				#ifdef LED_INFO_ENABLE  // ericyang 2017.02.16
					CHARGING_LED_OFF;
				#endif					
					stopCharging();
				#ifdef CHARGING_EXCEPTION_HANDLE_ENABLE
					initailizeChargerInfo();
					chargerInfo.status = connected;
					eventHandle.stopChargingFlag = true;  //notify msg to server 
				#endif
			//		eventHandle.stopChargingFlag = true;  // notify END charging msg to server; 
					//gChargingState = EV_IDLE;
					startS2SwitchOnOffCounterFlag = false;
          respMsgBuff.respCode = RESP_OK;
 					cmdMsgRespHandle(respMsgBuff.msgId);  //send respcond msg to server
					break;
			default:
					break;	
		}
}
#endif
/***********************************check charging time***************************/
void checkChargingTime(void)
{
		static int chargingTime = 0;
    if(chargerInfo.status == charging) {   // ericyang 20170111 modify
				if(chargingTime != chargerInfo.duration/60)
				{
					chargingTime = chargerInfo.duration/60;
					pc.printf("Current Charging Time:%d min, Total:%d min \r\n",chargingTime,chargerInfo.setDuration);
				}
        if((chargerInfo.duration/60) >= chargerInfo.setDuration) {
            chargerException.chargingTimeOverFlag = true;
        }
    }
}
/*****************************Check whether finish charging*************************/
void checkChargingFinish(void)
{
	static int chargingTime = 0;
	static float chargingEnergy = 0;
	if(chargerInfo.status == charging){
		if(chargerInfo.chargingType == CHARGING_TYPE_MONEY){
			if(fabs(chargingEnergy - chargerInfo.energy)>=0.1){
				chargingEnergy = chargerInfo.energy;
				pc.printf("Current charging energy:%0.1f kwh,Total energy:%0.1f kwh\r\n",chargingEnergy,chargerInfo.setEnergy);
			}
			if(chargerInfo.energy >= chargerInfo.setEnergy){
				chargerException.chargingEnergyOverFlag = true;
				chargingEnergy = 0;
				pc.printf("charging Energy over!\r\n");
			}
		}else if(chargerInfo.chargingType == CHARGING_TYPE_TIME){
			if(chargingTime != chargerInfo.duration/60){
				chargingTime = chargerInfo.duration/60;
				pc.printf("Current charging time:%d min,Total Time:%d min\r\n",chargingTime,chargerInfo.setDuration);
			}
			if(chargerInfo.duration/60 >= chargerInfo.setDuration){
				chargerException.chargingTimeOverFlag = true;
				chargingTime = 0;
				pc.printf("charging Time Over!\r\n");
			}
		}
	}
}

/*********************************get status of leds********************************/
char getledStatus(void)
{
	char status = 0;
	if((chargerInfo.status & 0x70) && (chargerInfo.status & errorCode))
    status = 0x03;
	else if(chargerInfo.status == charging)
		status = 0x02;
	else
		status = 0x01; 
  if(chargerInfo.connect == 1 || chargerInfo.connect == 2)
    status |= 0x04;	
	return status;
}
/***********************************check network connection***************************/
void checkNetworkAvailable(void)
{
    char heartbeatBuffer[64]; // send heartbeat package with json format,modified by orangeCai in 20170328 
    bool resendFlag = false;
#ifdef DISABLE_NETWORK_CONMUNICATION_FUNC
    if(eventHandle.stopCommunicationFlag == true)
        return;
#endif
    if(chargerException.serverConnectedFlag == true) {
        if(eventHandle.heatbeatFlag == true) {
            eventHandle.heatbeatFlag = false;
					  sprintf(heartbeatBuffer,NOTIFY_REQ_HeartPackage,getledStatus());
            if(tcpsocket.send(heartbeatBuffer,strlen(heartbeatBuffer)) <= 0) {
                pc.printf("send heartbeat packet error!\r\n");
                chargerException.serverConnectedFlag = false;
#ifdef LCD_ENABLE
                lcdshowonoffstate(false);
#endif
            } else {
                pc.printf("send heartbeat packet successful!\r\n");
							  pc.printf("send %d bytes,%s\r\n",strlen(heartbeatBuffer),heartbeatBuffer);
							  #ifdef NETWORK_COUNT_ENABLE
								pc.printf("numbers of sended massage:%d\r\n",++networkCount.sendCnt);
								#endif 
            }
        }
#ifdef LCD_ENABLE
            lcdshowonoffstate(false);
#endif				
/*				
        if(tcpsocket.is_connected() == false) { // fast check if server is ok
            chargerException.serverConnectedFlag = false;
					  pc.printf("never connect to server before!\r\n");
        }
*/				
    } else {
        tcpsocket.close();
        if(tcpsocket.connect(ECHO_SERVER_ADDRESS, ECHO_SERVER_PORT) < 0) {
            //     printf("Unable to connect 2 to (%s) on port (%d)\r\n", ECHO_SERVER_ADDRESS, ECHO_SERVER_PORT);
        } else {
						pc.printf("connected to server!\r\n");
            chargerException.serverConnectedFlag = true;
					  pc.printf("resetTimer = %d\r\n",resetTimer);
            if(eventHandle.firstConnectFlag != true)
							resendFlag = true;
#ifdef LCD_ENABLE
            lcdshowonoffstate(true);
#endif
        }
    }

    if(chargerException.serverConnectedFlag == false) {
        pc.printf("network unavailable!\r\n");
				#ifdef LED_INFO_ENABLE  // ericyang 20170120
				SERVER_CONNECT_LED_OFF;
				#endif
    }
		#ifdef LED_INFO_ENABLE   // ericyang 20170120
		else {
				SERVER_CONNECT_LED_ON;
		}
		#endif 
    
    if(resendFlag == true) {
        notifyMsgSendHandle(notifyNewDevice);
        resendFlag = false;
        OTAInit();//ericyang 20160920
    }
}

/********************************************recvMsgHandle()*****************************/
void recvMsgHandle(void)
{
    int len;
    // int n = tcpsocket.receive_all(socketInfo.inBuffer, len);
    len = sizeof(socketInfo.inBuffer);
    memset(socketInfo.inBuffer,0x0,len);
    // int n = tcpsocket.receive(socketInfo.inBuffer, len);
    int n = tcpsocket.receive_all(socketInfo.inBuffer, len);
    if(n > 0) {
        socketInfo.inBuffer[n] = '\0';
        pc.printf("receive %d bytes\r\n", n);
        pc.printf("receive %d bytes: %s \r\n", n,socketInfo.inBuffer);
        parseRecvMsgInfo(socketInfo.inBuffer);
			  
			  #ifdef NETWORK_COUNT_ENABLE
			  pc.printf("numbers of recieved massages:%d\r\n",++networkCount.recCnt); 
			  #endif
    }
}
/*******************************ENTER FACTORY MODE****************************************/
void enterFactoryMode(void)   // LOCAL IP: 192.168.1.101:13348
{
    UDPSocket udpsocket;
    Endpoint buddy;
    int len;
    char serverInfo[8];// 4 bytes IP, 2 bytes Port(H First), 2 bytes for crc16 checksum

    pc.printf("enter factory mode...\r\n");

    eth.init("192.168.1.101", "255.255.255.0", "192.168.1.1"); //set up static IP
    eth.connect();
    pc.printf("MAC Addr:%s\r\nIP Addr:%s\r\n",eth.getMACAddress(),eth.getIPAddress());    //get client IP address

    udpsocket.bind(13348);
    udpsocket.set_blocking(false, 1500);

    while (1) {
        memset(socketInfo.inBuffer,0x0,sizeof(socketInfo.inBuffer));
        memset(serverInfo,0x0,sizeof(serverInfo));
        if ((len = udpsocket.receiveFrom(buddy,socketInfo.inBuffer,sizeof(socketInfo.inBuffer))) > 0) {
            char *ip = buddy.get_address();
            int port = buddy.get_port();
            int serverPort = 0;
            uint16_t crc16 = 0;
            pc.printf("received from %s:%d len= %d,%s\r\n", ip, port, len,socketInfo.inBuffer);
            sscanf(socketInfo.inBuffer,"%d.%d.%d.%d:%d",serverInfo,serverInfo+1,serverInfo+2,serverInfo+3,&serverPort);
            serverInfo[4] = (serverPort >> 8) & 0xff;
            serverInfo[5] = serverPort & 0xff;
            crc16 = calculate_crc16(serverInfo, 6);
            serverInfo[6] = (crc16 >> 8) & 0xff;
            serverInfo[7] = crc16 & 0xff;
            if((serverInfo[0] == 0 || serverInfo[0] == 0xFF )
                    || (serverInfo[1] == 0 || serverInfo[1] == 0xFF )
                    || (serverInfo[2] == 0 || serverInfo[2] == 0xFF )
                    || (serverInfo[3] == 0 || serverInfo[3] == 0xFF )
                    || (serverPort == 0))

            {
                sprintf(socketInfo.outBuffer,"Unavailable Server Address: %d.%d.%d.%d port:%d Write Failed, Please try again!\r\n",serverInfo[0],serverInfo[1],serverInfo[2],serverInfo[3],(serverInfo[4] << 8) | serverInfo[5]);
            } else {
                erase_sector(SERVER_IP_ADDRESS);
                program_flash(SERVER_IP_ADDRESS,serverInfo, 8);// length must be a multiple of 8 when K64F
                sprintf(socketInfo.outBuffer,"Set Server Address: %d.%d.%d.%d port:%d OK! \r\nCharger MAC Address: %s\r\n",serverInfo[0],serverInfo[1],serverInfo[2],serverInfo[3],(serverInfo[4] << 8) | serverInfo[5],eth.getMACAddress());

            }
            udpsocket.sendTo(buddy,socketInfo.outBuffer,strlen(socketInfo.outBuffer));
        }
    }
}

/******************************exception Handle thread***************************/
void exceptionHandleThread(void const *argument)
{
//	uint32_t temp = us_ticker_read();
	while(true){
//		pc.printf("period:%d\r\n",(us_ticker_read()-temp)/1000);
//		temp = us_ticker_read();
		
//		checkChargerConnectStatus();
//		#ifdef GB18487_1_2015_AUTH_FUNC
//				checkChargingStatus();
//		#endif

//    chargingExceptionHandle();
//		if(eventHandle.checkMeterFlag == true) {
//        eventHandle.checkMeterFlag = false;
//        getMeterInfo();
//    }
//			pc.printf("period:%d ms\r\n",(us_ticker_read()-temp)/1000);
//			temp = us_ticker_read();		
			checkChargerConnectStatus();
			#ifdef GB18487_1_2015_AUTH_FUNC
				checkChargingStatus();
			#endif

			chargingExceptionHandle();		
		
		if(eventHandle.checkMeterFlag == false){
			Thread::wait(20);
		}else{
			eventHandle.checkMeterFlag = false;
      getMeterInfo();
		}
	}
}
/********************************************MAIN****************************************/
int main(void)
{
#ifdef LED_INFO_ENABLE  // ericyang 20161228
	WARNING_LED_OFF;
	CHARGING_LED_OFF;
	CONNECT_LED_OFF;
	SERVER_CONNECT_LED_OFF; // ericyang 20170215
#endif	

#ifdef EEPROM_ENABLE
		pinConfig();     //initialize i2c1 pins
#endif	
    pc.baud(115200); //initialize the PC interface
    RS485.baud(9600);
    sw2.rise(&sw2_release);  // sw2 interrupt

    switchRelayOnoff = RELAY_OFF;  // ericyang 20170111
    pc.printf("\r\nAPP VER: %s\r\n",VERSION_INFO);//20161009

    if(factoryMode == 0) {// PTD7 LOW
        enterFactoryMode();
    }
//#define EEPROM_TEST
#ifdef EEPROM_ENABLE

#ifdef EEPROM_TEST
#if 0
		char dataTest[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
		uint8_t addr = 0x92;
		pc.printf("addr:%x\r\n",addr);
		pc.printf("size of dataTest:%d\r\n",sizeof(dataTest));
		writeToEEPROM(addr,dataTest,sizeof(dataTest));
		readFromEEPROM(addr,tempBuffer,sizeof(dataTest));
		pc.printf("data:%s\r\n",tempBuffer);
#else
    uint8_t addr = 0x52;
		pc.printf("addr:%x\r\n",addr);		
		char meterNumber[9] = "12131415";
		ChargerInfo tmpChargerInfo;
		chargerInfo.connect = 1;
		chargerInfo.current = 14.5;
		chargerInfo.voltage = 233.3;
		chargerInfo.duration = 34;
		chargerInfo.energy = 13.5;
		chargerInfo.setDuration = 120;
		chargerInfo.power = 3133;
		chargerInfo.status = idle;
		memcpy(chargerInfo.meterNumber,meterNumber,sizeof(chargerInfo.meterNumber));
#if 1		
		writeToEEPROM(addr,(char*)(&chargerInfo),sizeof(chargerInfo));
		readFromEEPROM(addr,(char*)(&tmpChargerInfo),sizeof(tmpChargerInfo));	
		pc.printf("connet=%d\r\nstatus=%d\r\ncurrent=%f\r\nvoltage=%f\r\nenergy=%f\r\npower=%f\r\nmeterNumber=%s\r\n"
		,tmpChargerInfo.connect,tmpChargerInfo.status,tmpChargerInfo.current,tmpChargerInfo.voltage,
		tmpChargerInfo.energy,tmpChargerInfo.power,tmpChargerInfo.meterNumber);
#else
		uint16_t checksum,crc16;
		char* temp = (char*)malloc(sizeof(chargerInfo)+2);
		char* cdata;
		crc16 = calculate_crc16((char*)&chargerInfo,sizeof(chargerInfo));
		pc.printf("crc16=%x\r\n",crc16);
		temp[0] = crc16 >> 8;
		temp[1] = crc16 & 0xFF;
		memcpy(temp+2,(char*)&chargerInfo,sizeof(chargerInfo));
		writeToEEPROM(addr,temp,sizeof(chargerInfo)+2);
		readFromEEPROM(addr,cdata,sizeof(chargerInfo)+2);
		checksum = (cdata[0]<<8)|cdata[1];
		pc.printf("checksum=%x\r\n",checksum);
#endif

#endif
		while(1){}
#endif
#endif

#ifdef WDOG_ENABLE
		wDogInit(WDOG_TIMEOUT_VALUE_MS);
#endif    
#ifndef NOT_CHECK_NETWORK
    initETH();  //initialize the Ethernet connection
#endif
#ifdef LCD_ENABLE
    lcd.cls();
#endif

    disableCPPWM();  // ericyang 20160920 init pwm freq:0  duty:1;
    initServer();
    initCharger();  //ericyang 20160801
    initEventHandle(); // ericyang 20160920
#ifndef NOT_CHECK_NETWORK // ericyang 20160927    
    OTAInit();  // 20160912 for OTA FUNC
#endif
		us_ticker_init();
#ifdef RTC_ENABLE
rtcInit(RTC_INIT_TIME);
#ifdef RTC_TEST
		struct tm initTime;
		initTime.tm_year = 117;
		initTime.tm_mon = 5;
		initTime.tm_mday = 20;
		initTime.tm_hour = 0;
		initTime.tm_min = 0;
		initTime.tm_sec = 0; 
		set_time(mktime(&initTime));
		us_ticker_init();  
		pc.printf("rtc enable!\r\n");
#endif
#endif

#ifndef GB18487_1_2015_AUTH_FUNC
		if(chargerInfo.connect == CONNECTED_6V)
		{
			blue = BLUE_ON;  // ericyang 20161228 1->0
			chargerException.chargingEnableFlag == true;
			#ifdef CHARGING_CURRENT_32A
			enableCPPWM(PWM_DUTY_CURRENT_32A);  // 32A current
			pc.printf("sw2 =  1, enable PWM! PWM duty:%f\r\n",PWM_DUTY_CURRENT_32A);
			#endif
			#ifdef CHARGING_CURRENT_16A
			enableCPPWM(PWM_DUTY_CURRENT_16A);
			pc.printf("sw2 =  1, enable PWM! PWM duty:%f\r\n",PWM_DUTY_CURRENT_16A);
			#endif
		}
		else{
			blue = BLUE_OFF;  // ericyang 20161228 0->1
			chargerException.chargingEnableFlag == false;
			disableCPPWM();// ericyang 20161230
			pc.printf("sw2 =  0, disable PWM!\r\n");
		}
#endif		
//    tcpsocket.set_blocking(false, 1500);//??????
		tcpsocket.set_blocking(false,40);// depends on network delay 

//   pc.printf("current_sp:  %08x\r\n",__current_sp());
//    pc.printf("current_pc:  %08x\r\n",__current_pc());
		
    Thread th1(timerOneSecondThread,NULL,osPriorityNormal,/*1024 512*/1024);
    Thread th2(heartbeatThread,NULL,osPriorityNormal,/*1024 512*/512);
		Thread th3(exceptionHandleThread,NULL,osPriorityAboveNormal,1024);

    while (1) {
#ifdef  WDOG_ENABLE
			  wDogFeed();   //feed dog
#endif
		
#ifndef NOT_CHECK_NETWORK
        checkNetworkAvailable();
#endif
//       checkChargerConnectStatus(); // pwm 6v 12v detect

//#ifdef GB18487_1_2015_AUTH_FUNC
////				checkChargingStatus();  // ericyang 20170111
//#endif
//        checkChargingTime();
//			  checkChargingFinish();
//        if(eventHandle.checkMeterFlag == true) {
//          eventHandle.checkMeterFlag = false;
//          getMeterInfo();
//        }
//        chargingExceptionHandle();

#ifdef ENABLE_FULL_CHARGING_DETECT
				if(chargerInfo.status == charging && fullChargingFlag == true){
					fullChargingFlag = false;
					stopCharging();
				  #ifdef GB18487_1_2015_AUTH_FUNC 
	         disableCPPWM();
				  #endif
					chargerInfo.status = connected;
					#ifdef CHARGING_EXCEPTION_HANDLE_ENABLE
					initailizeChargerInfo();
					#endif
					eventHandle.stopChargingFlag = true; // notify END charging msg when reconnect to server
					chargingEndType = END_IN_ADVANCE_FULL;
					pc.printf("full of energy,stop charging!\r\n");
				}
#endif				

        if(chargerException.serverConnectedFlag == false) {
#ifndef NOT_CHECK_NETWORK
            if(resetTimer > RESET_TIMER) { // 2 minutes
                pc.printf("network error! system will be reset now, please wait...\r\n");
                NVIC_SystemReset();
            }
#endif
        } else {
            resetTimer = 0;

            if(notifySendID != invalidID) {
                if((systemTimer-notifySendCounter) >= SOCKET_RESEND_TIME)
								{
                   notifyMsgSendHandle(notifySendID);  // >5s not receive resp msg, resend notify
								   #ifdef NETWORK_COUNT_ENABLE
								  	pc.printf("numbers of resended massages:%d\r\n",++networkCount.reSendCnt);
								   #endif
								}
            } else {
							  if(eventHandle.firstConnectFlag == true){
									  pc.printf("first connetion!\r\n");
										eventHandle.firstConnectFlag = false;
									  notifyMsgSendHandle(notifyNewDevice);
								}if(eventHandle.stopChargingFlag == true) {
                    eventHandle.stopChargingFlag = false;
                    notifyMsgSendHandle(notifyEndCharging);
                }else if(eventHandle.updateChargerStatusFlag == true) {
                    eventHandle.updateChargerStatusFlag = false;
                    notifyMsgSendHandle(notifyChargerStatus);
                }else if(eventHandle.updateChargerInfoFlag == true) {
                    eventHandle.updateChargerInfoFlag = false;
										if(chargerInfo.status == charging) {  // ericyang 20170111
                        notifyMsgSendHandle(notifyChargingInfo);
                    }
                }else if(eventHandle.getLatestFWFromServerFlag == true) {
                    notifyMsgSendHandle(notifyUpdateVersion);
                }else if(eventHandle.updateVersionDoneFlag == true || eventHandle.updataVersionFailFlag == true){  //modify by orangeCai , 20170720
									  eventHandle.updateVersionDoneFlag = false;
									  eventHandle.updataVersionFailFlag = false;
									  notifyMsgSendHandle(notifyOTAResult);
								}
            }
            recvMsgHandle();
						/* detect whether to update code,orangeCai 20170602 */
						if(updateCodeFlag == true && (chargerInfo.status & CHARGER_STATUS_MASK) != charging ){ //update code while not charging
							updateCodeFlag = false;
							pc.printf("\r\n\r\n chargerInfo.status = %d,start to update code!\r\n\r\n",chargerInfo.status);
							tcpsocket.close(); //close socket,20170713
							updateCode(); // begin to update code
						}
        }
    }
    eth.disconnect();   //close Ethernet
}   //end main()