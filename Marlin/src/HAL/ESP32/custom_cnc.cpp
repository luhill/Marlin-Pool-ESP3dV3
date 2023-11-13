#include "custom_cnc.h"
#include <driver/ledc.h> //used for pwm
#include <EEPROM.h>
#include <time.h>
#include <sys/time.h>
#include "esp_sntp.h"
#include "../../core/serial.h"
#include "../../core/macros.h"
#include "../../pins/esp32/pins_MKS_TINYBEE.h"



//#define _TIMERINTERRUPT_LOGLEVEL_     4
#include "ESP32TimerInterrupt.h"

#define EEPROM_SETTINGS_LOCATION (MARLIN_EEPROM_SIZE-400)
/** MKS TINYBEE expansion plug pins
 *                ------                                 ------
 *  (BEEPER) 149 | 1  2 | 13 (BTN_ENC)    (SPI MISO) 19 | 1  2 | 18 (SPI SCK)
 *  (LCD_EN)  21 | 3  4 |  4 (LCD_RS)      (BTN_EN1) 14 | 3  4 |  5 (SPI CS)
 *  (LCD_D4)   0   5  6 | 16 (LCD_D5)      (BTN_EN2) 12   5  6 | 23 (SPI MOSI)
 *  (LCD_D6)  15 | 7  8 | 17 (LCD_D7)      (SPI_DET) 34 | 7  8 | RESET
 *           GND | 9 10 | 5V                        GND | 9 10 | 3.3V
 *                ------                                 ------
 *                 EXP1                                   EXP2
 */

//Pin Definitions: #define PIN_NAME   pin_number

struct Settings{//26 bytes
  byte initialized;//1 byte = flag, used to determine if the eeprom has been initialized with our settings yet
  byte auto_on;//1 byte = flag
  int time_start;//4 bytes = int
  int time_stop;//4 bytes = int
};
#define SEC_PER_DAY 86400
static Settings S;

static volatile bool updateWebUi = false;

static const char j_start [] = "{\"myPanel\":{\"values\":{";
static const char j_end [] = "}}}\n";
static const char j_time_current [] = "\"time_current\":\"%s\"";
static const char j_auto [] = "\"auto\":[%i,%i,%i]";

static const char j_ui [] = "{\"myPanel\":{\"name\":\"CUSTOM_CNC \",\"ui\":{"
            "\"time_current\":{"
            "\"type\":\"datetime-local\","
            "\"label\":\"Time\","
            "\"cmd\":\"P0\""
            "},\"auto\":{"
            "\"type\":\"onStartStop\","
            "\"label\":\"Auto\","
            "\"cmd\":\"P3\""
            "}}}}\n";
//char arrays to format and send json to the web interface
static char status[250];
static char sendc[250];

//Timer library that creates up to 16 timers from a single hardware timer
ESP32Timer hTimer3(3); //The timer library will use hardware timer 3 which is also used for tone() or beepers by the tinybee
ESP32_ISRTimer ISR_Timer;

static int ISR_TIMER_AUTO_ON;
static int ISR_TIMER_AUTO_OFF;

void IRAM_ATTR alarm_autoOn(){
  if(S.auto_on){
    updateWebUi = true;
    //todo: task when on triggered
    CUSTOM_CNC::writeOutputs();
    ISR_Timer.changeInterval(ISR_TIMER_AUTO_ON,86400000);//resquedule the timer for 24hrs (initial setup was unlikely 24 hr interval)
  }
}
void IRAM_ATTR alarm_autoOff(){
  if(S.auto_on){
    updateWebUi = true;
    //todo: task when off triggered
    CUSTOM_CNC::writeOutputs();
    ISR_Timer.changeInterval(ISR_TIMER_AUTO_OFF,86400000);
  }
}

//ISR_timer ticks 
bool IRAM_ATTR hwTimerHandler(void * timerNo){
	ISR_Timer.run();
	return true;
}
CUSTOM_CNC::CUSTOM_CNC(){}

static int onS, dutyS, startS, stopS;//used in switch statment
void CUSTOM_CNC::handleCommand(const int8_t c, String val){
    switch (c){
      /*extract value from string value:
      val.toInt()
      or scan in multiple values:
      sscanf(val.c_str(),"[%u,%u,%u]",&onS, &startS, &stopS);

      update the webui via serial. serial message composed like below
      sprintf(status, j_auto_a,S.auto_on_a,S.time_start_a, S.time_stop_a);
      */
    case 0://current time
        if(val!=""){setTimeCurrent(val.c_str());}
        sprintf(status, j_time_current, getTimeCurrent());
        break;
    case 1://Pump on off
        break;
    case 3://Auot timer
        sscanf(val.c_str(),"[%u,%u,%u]",&onS, &startS, &stopS);
        setAuto(onS>0,startS,stopS);
        sprintf(status, j_auto,S.auto_on,S.time_start, S.time_stop);
        break;
    case 113://update ui
        SERIAL_ECHO(j_ui);
        return;
    case 114://web interface requesting all settings
        snprintf(sendc,sizeof(sendc),"%s,%s",j_time_current, j_auto);
        snprintf(status,sizeof(status),sendc,getTimeCurrent(),S.auto_on,S.time_start,S.time_stop);
        break;
    default:
        SERIAL_ECHOLN("unknown CUSTOM_CNC command");
        return;
        break;
    }
    snprintf(sendc,sizeof(sendc),"%s%s%s",j_start, status, j_end);
    SERIAL_ECHOLN(sendc);
}

void CUSTOM_CNC::setup(){
    //Set Pin states

    setupClock();
    loadSettings();//get saved settings from eeprom
    if(!S.initialized){//the settings have never been saved (code loaded to device for the first time or eeprom was erased)
      S.initialized = true;
      S.auto_on = false;
      S.time_start = 30600;//8:30 am
      S.time_stop = 55800;//60*15.5;//3:30 pm
      saveSettings();
    }
   
    if (hTimer3.attachInterruptInterval(1000,hwTimerHandler)){}//set hardware timer to run at 1ms (1000 us)
    //Up to 16 timers based on hTimer3. These timers tick length is determined by hTimer3 interrupt interval
    /*Note - reading an i2s analog to digital converter cannot be performed inside an ISR*/
    ISR_TIMER_AUTO_ON = ISR_Timer.setInterval(SEC_PER_DAY*1000,alarm_autoOn);                      //auto on timer
    ISR_TIMER_AUTO_OFF = ISR_Timer.setInterval(SEC_PER_DAY*1000,alarm_autoOff);                    //auto off timer
    
    setAutoTimers();//Auto on & off timers were initialized with arbitrary values above. Update them based on current time
    
    writeOutputs();
}

void CUSTOM_CNC::loop(){
  if(updateWebUi){
    updateWebUi = false;
    CUSTOM_CNC::handleCommand(1,"");//auto mode has turned the equipment on or off. Update the UI
  }
  return;
}

void CUSTOM_CNC::setupClock(){
    sntp_set_time_sync_notification_cb(timeSyncCallback);//time server may take a 
    configTime(10*60*60, 0, "au.pool.ntp.org","pool.ntp.org");//brisbane time 10hrs ahead , 0 daylight saving
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){//error getting time from ntp server. set it manually
      timeinfo.tm_year = 2023 - 1900;
      timeinfo.tm_mon = 8;
      timeinfo.tm_mday = 6;
      timeinfo.tm_hour = 12;
      timeinfo.tm_min = 0;
      timeinfo.tm_sec = 0;
      time_t t = mktime(&timeinfo);
      struct timeval now = { .tv_sec = t };
      settimeofday(&now, NULL);
    }
    return;
}
void CUSTOM_CNC::timeSyncCallback(struct timeval *tv){
    setAutoTimers();
    CUSTOM_CNC::handleCommand(0,"");//output time to serial 
}
void CUSTOM_CNC::setAutoTimers(){
    time_t now;
    time(&now);
    struct tm lTime;
    localtime_r(&now,&lTime);
    int sec = (lTime.tm_hour*3600 + lTime.tm_min*60 + lTime.tm_sec);
  
    int start = (S.time_start-sec+SEC_PER_DAY)%SEC_PER_DAY;
    int stop = (S.time_stop-sec+SEC_PER_DAY)%SEC_PER_DAY;
    ISR_Timer.changeInterval(ISR_TIMER_AUTO_ON,start*1000);
    ISR_Timer.changeInterval(ISR_TIMER_AUTO_OFF,stop*1000);
}

void CUSTOM_CNC::setAuto(bool on, int start, int stop){
    bool changed = false;
    if(on != S.auto_on){
      S.auto_on = on;
      changed = true;
    }
    if(start != S.time_start){
      S.time_start = start;
      changed = true;
    }
    if(stop != S.time_stop){
      S.time_stop = stop;
      changed = true;
    }
    if(changed){
      setAutoTimers();
      saveSettings();//save to eeprom
    }
}

void CUSTOM_CNC::writeOutputs(){
  //Do not use 'digitalWrite' use macros defined in ESP32/fastio.h ('WRITE') as some mks tinybee pins are virtual shift register pins
  //WRITE(PIN_NAME,HIGH)
}

void CUSTOM_CNC::setTimeCurrent(const char *sDateTime){//2023-09-11T16:30:21
  struct tm ti;
  sscanf(sDateTime,"%d-%d-%dT%d:%d",&ti.tm_year,&ti.tm_mon,&ti.tm_mday,&ti.tm_hour,&ti.tm_min);
  ti.tm_year-=1900;
  ti.tm_mon-=1;//0-11
  time_t t = mktime(&ti);
  struct timeval now = { .tv_sec = t };
  settimeofday(&now, NULL);
  setAutoTimers();
}
char* CUSTOM_CNC::getTimeCurrent(){
    time_t now;
    time(&now);
    struct tm lTime;
    localtime_r(&now,&lTime);
    static char dts[22];
    strftime(dts, sizeof(dts), "%Y-%m-%dT%X", &lTime);
    return dts;
}
void CUSTOM_CNC::loadSettings(){
  EEPROM.begin(MARLIN_EEPROM_SIZE);
  EEPROM.get(EEPROM_SETTINGS_LOCATION,S);
  EEPROM.end();
}
void CUSTOM_CNC::saveSettings(){
  if (!EEPROM.begin(MARLIN_EEPROM_SIZE)){
      SERIAL_ECHO("eeprom failed to start");
  }
  EEPROM.put(EEPROM_SETTINGS_LOCATION,S);
  EEPROM.end();
}

