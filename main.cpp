/**
	---------------------------------------------------------------------------
	* Describtion:            Firmware for AC EV Charger
	* Company:                WPI-ATU
	* Author:                 Orange
	* Version:                V2.0
	* Date:                   2018-03-16
	* Function List:
			1. RS485 Communication
			2. Ethernet Communication,RJ45 interface
	    3. Wechat control
			4. Follow the GB/T 18487.1-2015 Standard
			5. Use Mbed OS 2.0
	* History:
	---------------------------------------------------------------------------
**/

/*----------------------------------------------------------------------------
 * Includes
-----------------------------------------------------------------------------*/
#include "UserConfig.h"
#include "EVCharger.h"
#include "cJSON.h"
#include "lib_crc16.h"
#include "flashLayout.h"
#include "otaCharger.h"

#ifdef WDOG_ENABLE
#include "WatchDog.h"
#endif

#ifdef EEPROM_ENABLE
extern "C"{
	#include "MK64F12.h"
}
#endif

/*----------------------------------------------------------------------------
 * Macro Definitions
-----------------------------------------------------------------------------*/
#define    RESET_TIMER      120

/*----------------------------------------------------------------------------
 * Global Variable
-----------------------------------------------------------------------------*/

const char* DEFAULT_SERVER_ADDRESS = "112.74.170.197";
const int DEFAULT_SERVER_PORT = 22221;

char ECHO_SERVER_ADDRESS[20];
int ECHO_SERVER_PORT;

Serial pc(USBTX, USBRX);      //uart for debug
Serial RS485(PTE24,PTE25);  //uart for RS485
DigitalOut switchRelayOnoff(PTB23);//PTB23
AnalogIn cpADC(A0);  // adc for cp detect PTB2
PwmOut cpPWM(D6);  // cp pwm signal generator PTC2
EthernetInterface eth;          //create ethernet
TCPSocketConnection tcpsocket;  //create tcp socket

/* LED for indicate device status */
DigitalOut red(LED_RED);        //warning led
DigitalOut green(LED_GREEN);    //charging led
DigitalOut blue(LED_BLUE);    //connect led
DigitalOut server(PTB20);    //server connected led

DigitalIn factoryMode(PTD7);  //for entering factory mode

#ifdef EEPROM_ENABLE
I2C i2c1(PTC11,PTC10);    //orangecai I2C1 Read Epprom data
#endif

Mutex myMutex;
char EVChargerModelNO[20]; // set MAC as EVCharger Model NO
SystemEventHandle eventHandle;
SocketInfo socketInfo;
int systemTimer = 0;
int resetTimer = 0;
float startEnergyReadFromMeter = 0;
float totalEnergyReadFromMeter = 0;
bool pwmState = false;

volatile bool checkChargingFinishFlag = false;
volatile bool pauseChargingFlag = false;
volatile bool startS2SwitchWaitCounterFlag = false;
volatile int pauseChargingCounter = 0;
volatile int waitS2SwitchOnCounter = 0;

volatile bool S2SwitchEnabledFlag = false;
volatile bool startS2SwitchOnOffCounterFlag = false;
volatile int S2SwitchOnOffCounter = 0;
volatile int gChargingState = EV_IDLE;

char tempBuffer[256];
char dataBuffer[256];
#ifdef RTC_ENABLE
	char timeStampMs[32];
#endif

#ifdef NETWORK_COUNT_ENABLE
volatile NetworkCount networkCount = {0,0,0}; 	
#endif	

/*the flag would be set when exception happened while charging*/
volatile bool chargingExceptionFlag = false;
 /*the flag would be set when exception disappeared while charging*/
volatile bool rechargingFlag = false;
volatile bool startContinueChargingFlag = false; 
volatile uint32_t chargingErrorTimer = 0;
volatile uint32_t rechargingTimer = 0;
volatile uint8_t rechargingWaitCounter = 0;

 /*Update code while the flag is set */
volatile bool updateCodeFlag = false;
 /* true:OTA code download success */
 volatile bool otaSuccessFlag = false;

#ifdef ENABLE_FULL_CHARGING_DETECT
	volatile uint16_t fullChargingDetectCounter = 0;
	volatile bool fullChargingFlag = false;
#endif

/*----------------------------------------------------------------------------
 * Function declaration
-----------------------------------------------------------------------------*/
void initServer(void);
void initETH(void);
void enterFactoryMode(void);
void timerOneSecondThread(void const *argument);
void heartbeatThread(void const *argument);
void checkNetworkAvailable(void);
void checkChargingTime(void);
void checkChargerConnectStatus(void);
int getMeterInfo(void);
void sw2_release(void);
CPSignal checkCPSignal(void);

/*----------------------------------------------------------------------------
 * Function Definition
-----------------------------------------------------------------------------*/
#ifdef EEPROM_ENABLE
/*!
	@birfe Config pins for I2C1
	@input None
	@output None
*/
static void pinConfig(void)
{
	/* config I2C1 SCL pin , PTC10 */
	PORTC->PCR[10] |= (uint32_t)0x03; 
	/* config I2C1 SDA pin , PTC11 */
	PORTC->PCR[11] |= (uint32_t)0x03;
}
#endif

#ifdef RTC_ENABLE
/*!
	@birfe get current RTC time
	@input Pointer to string for storing the RTC time
	@output None
*/
void getTimeStampMs(char* timeStampMs)
{
  time_t seconds;
	char buffer[32];
	seconds = rtc_read();
	strftime(buffer, 32, "%y%m%d%H%M%S", localtime(&seconds));
	sprintf(timeStampMs,"%s%d",buffer,us_ticker_read()%1000000/1000);
}
/*!
	@birfe Init RTC Function
	@input init value for RTC, 
	@output None
*/
void rtcInit(uint32_t timeSeconds)
{
	char* initTime;
	set_time(timeSeconds);  //time in seconds since January 1, 1970
	initTime = ctime(&timeSeconds);
	pc.printf("init time:%s\r\n",initTime);
	us_ticker_init(); 
}

#endif

/*!
	@birfe Init Ethernet interface,used DHCP
	@input None
	@output None
*/
void initETH(void)
{
    uint8_t MAC_ADDRESS[6];
    eth.init(); //DHCP
    eth.connect();  //connect ethernet
    pc.printf("MAC Addr:%s\r\nIP Addr:%s\r\n",eth.getMACAddress(),eth.getIPAddress());//get client IP address
    if(strcmp(eth.getIPAddress(),NULL) == NULL) {
        pc.printf("RJ45 error! system will be reset now, please wait...\r\n");
        NVIC_SystemReset();
    }
    sscanf(eth.getMACAddress(),"%02x:%02x:%02x:%02x:%02x:%02x",
		&MAC_ADDRESS[0],&MAC_ADDRESS[1],&MAC_ADDRESS[2],&MAC_ADDRESS[3],&MAC_ADDRESS[4],&MAC_ADDRESS[5]);
    sprintf(EVChargerModelNO,"%02x%02x%02x%02x%02x%02x",
		MAC_ADDRESS[0],MAC_ADDRESS[1],MAC_ADDRESS[2],MAC_ADDRESS[3],MAC_ADDRESS[4],MAC_ADDRESS[5]);
}

/*!
	@birfe Init server IP address
	@input None
	@output None
*/
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

/*!
	@birfe Init eventHandle structure
	@input None
	@output None
*/
void initEventHandle(void)
{
    eventHandle.heatbeatFlag = false;
    eventHandle.checkMeterFlag = false;
    eventHandle.updateChargerInfoFlag = false;
    eventHandle.updateChargerStatusFlag = false;
    eventHandle.stopChargingFlag = false;
    eventHandle.getLatestFWFromServerFlag = false;
	  eventHandle.updateVersionDoneFlag = false;
	  eventHandle.firstConnectFlag = true;
}

/*!
	@birfe 1s thread 
	@input None
	@output None
*/
void timerOneSecondThread(void const *argument)
{
    while (true) {	
			if(systemTimer >= 3){
						eventHandle.checkMeterFlag = true; // 1s check meter info
        }
				
        if(chargerException.serverConnectedFlag == false) {
            resetTimer++;
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

				if(rechargingFlag == true){
						if(rechargingTimer > NORMAL_TIME_HOLDED){
							  startContinueChargingFlag = true;
								rechargingFlag = false;
								rechargingWaitCounter = 0;
								rechargingTimer = 0;
						}else if(chargerInfo.status != connected){
								if(++rechargingWaitCounter > RECHARGING_WAIT_TIMES_ALLOWED){
									  rechargingWaitCounter = 0;
										eventHandle.stopChargingFlag = true; //notify END charging msg to server
									  chargingEndType = getChargingEndType(chargerInfo.status);
										initChargerInfo();
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
						  chargingErrorTimer = 0;
						  rechargingWaitCounter = 0;
						  eventHandle.stopChargingFlag = true;  //notify END charging msg to server
						  if(chargerInfo.status == 0x4082)
								chargerInfo.status = idle;
							else
								chargerInfo.status &= 0xFFFC;
						  chargingEndType = getChargingEndType(chargerInfo.status);
						  initChargerInfo();
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

				if(startS2SwitchOnOffCounterFlag == true)
						S2SwitchOnOffCounter++;

				if(systemTimer % 5 == 0)
					checkChargingFinishFlag = true;
        Thread::wait(1000);
        systemTimer++;
        if(chargerInfo.status == charging)
          chargerInfo.duration++;
				if(pauseChargingFlag == true){
					pauseChargingCounter++;
				}
				if(startS2SwitchWaitCounterFlag == true){
					waitS2SwitchOnCounter++;
				}
    }
} 

/*!
	@birfe heartbeat thread
	@input None
	@output None
*/
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

/*!
	@birfe Get meter information
	@input None
	@output None
*/
int getMeterInfo(void)
{
    char ModbusData[8] = {0x2C,0x03,0x00,0x00,0x00,0x07};// slave addr,function code,Hi PDU addr,low PDU addr,Hi N reg,Lo N reg,Lo CRC,Hi CRC
    char regvalue[20] = {0};
    uint16_t crc16;
    int i,count;
    static int meterErrorTimes = 0;
		static int voltageErrorTimes = 0;
		static int currentErrorTimes = 0;
		
    ModbusData[0] = atoi(chargerInfo.meterNumber + 6); // get last two number [6][7] as meter addr
    pc.printf("meter address: %x\r\n",ModbusData[0]);

    crc16 = calculate_crc16_Modbus(ModbusData, 6);
    ModbusData[6] = crc16 & 0xff;
    ModbusData[7] = (crc16 >> 8) & 0xff;

    //myMutex.lock();
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
    //myMutex.unlock();
    crc16 = calculate_crc16_Modbus(regvalue,17);
    if(( (crc16 & 0xff) == regvalue[17]) && (((crc16 >> 8) & 0xff) == regvalue[18])) {
        meterErrorTimes = 0;
        if(chargerException.meterCrashedFlag == true) {
            chargerException.meterCrashedFlag = false; // ericyang 20160824 meter recover ok
        }

        totalEnergyReadFromMeter = (regvalue[5]<<24 | regvalue[6]<<16 | regvalue[3]<<8 | regvalue[4])*0.1;
        chargerInfo.energy =  totalEnergyReadFromMeter - startEnergyReadFromMeter;
        chargerInfo.voltage = (regvalue[7]<<8 | regvalue[8])*0.01;
        chargerInfo.current = (regvalue[11]<<24 | regvalue[12]<<16 | regvalue[9]<<8 | regvalue[10])*0.001;
        chargerInfo.power = ((regvalue[15]&0x7F)<<24 | regvalue[16]<<16 | regvalue[13]<<8 | regvalue[14])*0.1;

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
					#ifdef CHARGING_CURRENT_16A
						if(chargerInfo.current > 18) // current: 0~18 13+2 >5s  fix 18487.1-2015 A3.10.7
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
            }
            pc.printf("\r\nMeter Crashed!!!\r\n");
        }
        return -1; // read meter error!!!
    }
    return 0;
}

/*!
	@birfe Disable CP PWM
	@input None
	@output None
*/
void disableCPPWM(void)
{
    cpPWM.period_ms(0);
    cpPWM = 1;
    pwmState = false;
}
/*!
	@birfe Enable CP PWM
	@input Duty,range is 0~1.0
	@output None
*/
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
/*!
	@birfe Detect CP signal,sample 20 times in 1s
	@input None
	@output None
*/
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
		for(i=0;i<ADC_SAMPLE_USE_TOP_NUM;i++){  //FIX_GB18487_TEST_PROBLEM
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
		if((averValue > 2.14)&&(averValue < 2.46)) { // (12-0.5) / 5  = 2.30 +/- (0.8/5)
				ret = PWM12V;
			} else if((averValue > 1.54)&&(averValue < 1.86)) { // (9-0.5) / 5  = 1.70 +/- (0.8/5)
				ret = PWM9V;
			}	else if((averValue > 0.94) && (averValue < 1.26)) { // (6-0.5) / 5  = 1.10 +/- (0.8/5)
				ret = PWM6V;
		}	else if(averValue < 0.1){
				ret = PWM0V;
		} else {
				ret = PWMOtherVoltage;
		}
//		float value = averValue*5+0.5f;
//		pc.printf("voltage:%f v\r\n",value);
    return ret;
}
/*!
	@birfe Check the connect status of charger,total 3 status,NOT_CONNECTED,CONNECTED_6V,CONNECTED_9V
	@input None
	@output None
*/
void checkChargerConnectStatus(void)
{
		static int connectStatus = NOT_CONNECTED;  // 20170216 fixed
    static CPSignal cpSignal = PWMNone;
		CPSignal tmpCPSignal = checkCPSignal();

		
		if(cpSignal != tmpCPSignal)
		{
			cpSignal = tmpCPSignal;
			pc.printf("1: 12V  2: 9V 3: 6V 4: 0V 5: Other cpSignal=%d\r\n",cpSignal);
		}

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
		}
		#ifdef LED_INFO_ENABLE		
		if(chargerException.chargingEnableFlag == true){
				CONNECT_LED_ON;
		}else{
				CONNECT_LED_OFF;
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
/*!
	@birfe Check charging status
	@input None
	@output None
*/
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
										stopCharging();
										pauseChargingFlag = false;
								}
								disableCPPWM();//GB/T 18487.1-2015 A3.10.9
								pc.printf("error connect, disable charging!\r\n");
								gChargingState = EV_IDLE;
						}else{
								if(pauseChargingFlag == true && pauseChargingCounter > MAX_PAUSE_TIME){ //timeout of pause charging
										pauseChargingFlag = false;
									  eventHandle.stopChargingFlag = true;
									  chargingEndType = END_IN_ADVANCE_FULL;
										pauseChargingCounter = 0;
										stopCharging();
									  disableCPPWM(); 
										initChargerInfo();
                    chargerInfo.status = connected;									

									  gChargingState = EV_IDLE;
								}
								if(startS2SwitchWaitCounterFlag == true && waitS2SwitchOnCounter > MAX_WAIT_TIME){
										startS2SwitchWaitCounterFlag = false;
										waitS2SwitchOnCounter = 0;
									  disableCPPWM();
										initChargerInfo();
										chargerInfo.status = connected;

									  gChargingState = EV_IDLE;
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
						#ifdef LED_INFO_ENABLE
							CHARGING_LED_OFF;
						#endif	
							if(chargerInfo.connect == CONNECTED_9V){  // S2 switch open  PWM still enable
									gChargingState = EV_CONNECTED_PRE_START_CHARGING;
									pauseCharging(true);
							}
							else{
									stopCharging();	  //gChargingState = EV_IDLE;
									disableCPPWM();
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
				#ifdef LED_INFO_ENABLE
					CHARGING_LED_OFF;
				#endif					
					stopCharging();
					initChargerInfo();
					chargerInfo.status = connected;
					eventHandle.stopChargingFlag = true;  //notify msg to server 
				
					startS2SwitchOnOffCounterFlag = false;
					break;
			default:
					break;	
		}
}

/*!
	@birfe Check whether charging is finished
	@input None
	@output None
*/
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

/*!
	@birfe get current LED status
	@input None
	@output None
*/
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
/*!
	@birfe Check current network condition
	@input None
	@output None
*/
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
        }
    }

    if(chargerException.serverConnectedFlag == false) {
        pc.printf("network unavailable!\r\n");
			#ifdef LED_INFO_ENABLE 
				SERVER_CONNECT_LED_OFF;
			#endif
    }else {
			#ifdef LED_INFO_ENABLE
				SERVER_CONNECT_LED_ON;
			#endif
		}
		 
    
    if(resendFlag == true) {
        notifyMsgSendHandle(notifyNewDevice);
        resendFlag = false;
        OTAInit();
    }
}

/*!
	@birfe Handle recieved massege
	@input None
	@output None
*/
void recvMsgHandle(void)
{
    int len;
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
/*!
	@birfe Enter factory mode,used UDP Protocol to config server IP address
	@input None
	@output None
*/
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

/*!
	@birfe Handle expection while charging
	@input None
	@output None
*/
void exceptionHandleThread(void const *argument)
{
	while(true){
			checkChargerConnectStatus();			
  		checkChargingStatus();
			chargingExceptionHandle();		

		if(eventHandle.checkMeterFlag == false){
			Thread::wait(20);
		}else{
			eventHandle.checkMeterFlag = false;
      getMeterInfo();
		}
	}
}
/*!
	@birfe network thread,handle massage transceiver
	@input None
	@output None
*/
void netWorkThread(void const *argument)
{
	while(true){
		        if(chargerException.serverConnectedFlag == false) {
							if(resetTimer > RESET_TIMER) { // 2 minutes
									pc.printf("network error! system will be reset now, please wait...\r\n");
									NVIC_SystemReset();
							}
        }else {
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
								}else if(eventHandle.stopChargingFlag == true){
                    eventHandle.stopChargingFlag = false;
                    notifyMsgSendHandle(notifyEndCharging);
                }else if(eventHandle.updateChargerStatusFlag == true) {
                    eventHandle.updateChargerStatusFlag = false;
                    notifyMsgSendHandle(notifyChargerStatus);
                }else if(eventHandle.updateChargerInfoFlag == true) {
                    eventHandle.updateChargerInfoFlag = false;
										if(chargerInfo.status == charging) {
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
						/* detect whether to update code*/
						if(updateCodeFlag == true && (chargerInfo.status & CHARGER_STATUS_MASK) != charging ){ //update code while not charging
							updateCodeFlag = false;
							pc.printf("\r\n\r\n chargerInfo.status = %d,start to update code!\r\n\r\n",chargerInfo.status);
							tcpsocket.close(); //close socket
							updateCode(); // begin to update code
						}
        }
			Thread::wait(50);
	}
}
/*!
	@birfe main function
	@input None
	@output None
*/
int main(void)
{
#ifdef LED_INFO_ENABLE
	WARNING_LED_OFF;
	CHARGING_LED_OFF;
	CONNECT_LED_OFF;
	SERVER_CONNECT_LED_OFF;
#endif	

#ifdef EEPROM_ENABLE
		pinConfig();     //initialize i2c1 pins
#endif	
    pc.baud(115200);
    RS485.baud(9600);

    switchRelayOnoff = RELAY_OFF;
    pc.printf("\r\nAPP VER: %s\r\n",VERSION_INFO);

    if(factoryMode == 0) {
        enterFactoryMode();
    }

#ifdef WDOG_ENABLE
		wDogInit(WDOG_TIMEOUT_VALUE_MS);
#endif    

    initETH();
    disableCPPWM();
    initServer();
    initCharger();
    initEventHandle();
    OTAInit();
		us_ticker_init();

		tcpsocket.set_blocking(false,40);// depends on network delay 
		
    Thread th1(timerOneSecondThread,NULL,osPriorityNormal,1024);
    Thread th2(heartbeatThread,NULL,osPriorityNormal,512);
		Thread th3(exceptionHandleThread,NULL,osPriorityAboveNormal,1024);
		Thread th4(netWorkThread,NULL,osPriorityNormal,1024);
    
    while (1) {
#ifdef  WDOG_ENABLE
			  wDogFeed();   //feed dog
#endif
		
			checkNetworkAvailable();
			if(checkChargingFinishFlag == true){
				checkChargingFinish();
				checkChargingFinishFlag = false;
			}
#ifdef ENABLE_FULL_CHARGING_DETECT
				if(chargerInfo.status == charging && fullChargingFlag == true){
					fullChargingFlag = false;
					stopCharging();
	        disableCPPWM();
					chargerInfo.status = connected;
					initChargerInfo();
					eventHandle.stopChargingFlag = true; // notify Endcharging msg when reconnect to server
					chargingEndType = END_IN_ADVANCE_FULL;
					pc.printf("full of energy,stop charging!\r\n");
				}
#endif				
    }
}