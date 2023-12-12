#include "chiliPad.h"
#include <driver/ledc.h>
#include <EEPROM.h>
#include <time.h>
#include <sys/time.h>
#include "esp_sntp.h"
#include "../../core/serial.h"
#include "../../core/macros.h"
#include <Adafruit_ADS1X15.h>
#include "../../pins/esp32/pins_MKS_TINYBEE.h"
#include "../../module/temperature.h"
#include "HAL.h"

//#define _TIMERINTERRUPT_LOGLEVEL_     4
#include "ESP32TimerInterrupt.h"

#define EEPROM_SETTINGS_LOCATION (MARLIN_EEPROM_SIZE-400)
/**
 *                ------                                 ------
 *  (BEEPER) 149 | 1  2 | 13 (BTN_ENC)    (SPI MISO) 19 | 1  2 | 18 (SPI SCK)
 *  (LCD_EN)  21 | 3  4 |  4 (LCD_RS)      (BTN_EN1) 14 | 3  4 |  5 (SPI CS)
 *  (LCD_D4)   0   5  6 | 16 (LCD_D5)      (BTN_EN2) 12   5  6 | 23 (SPI MOSI)
 *  (LCD_D6)  15 | 7  8 | 17 (LCD_D7)      (SPI_DET) 34 | 7  8 | RESET
 *           GND | 9 10 | 5V                        GND | 9 10 | 3.3V
 *                ------                                 ------
 *                 EXP1                                   EXP2
 */
//#define PIN_PUMP_POWER          EXP1_01_PIN
//#define PIN_BOOSTER_POWER       EXP1_03_PIN
#define PIN_PUMP_DUTY             EXP1_04_PIN
#define PIN_FAN_DUTY              EXP1_05_PIN
//#define PIN_ION_DUTY_FWD        EXP1_06_PIN
//#define PIN_ION_DUTY_REV        EXP1_07_PIN
//#define PIN_POWER_SUPPLY_ON       EXP1_08_PIN

//#define PIN_ACID                HEATER_0_PIN
//#define PIN_FLOC                HEATER_1_PIN
#define PIN_12V_PUMP              HEATER_BED_PIN
#define PIN_ON_INTERRUPT          Y_STOP_PIN
#define PIN_LEVEL_SWITCH          Z_STOP_PIN
//#define PIN_ON_INTERRUPT_B      Z_STOP_PIN

#define PIN_LED                   FAN_PIN
//#define PIN_LED_BOOSTER         FAN1_PIN

struct Settings{//26 bytes
  byte initialized;//1 byte = flag, used to determine if the eeprom has been initialized with CHILIPAD settings yet
  byte auto_on;//1 byte = flag
  int duty_pump;//4 bytes = int
  int duty_fan_max;//4 bytes = int
  int time_start;//4 bytes = int
  int time_stop;//4 bytes = int
  float temp_setPoint;
};
#define SEC_PER_DAY 86400
#define pumpPWMChannel 0
#define fanPWMChannel 1
#define ledPWMChannel 2

static Settings S;
static volatile bool pump_on = false;
static volatile bool fan_on = false;
static volatile bool led_on = false;
static volatile bool read_temps = true;
static volatile int duty_fan = 64;
static volatile bool updateWebUi = false;
static volatile float temp_water = 25;
static volatile float temp_air = 25;
static volatile float temp_other = 25;

static const char j_start [] = "{\"myPanel\":{\"values\":{";
static const char j_end [] = "}}}\n";
static const char j_time_current [] = "\"time_current\":\"%s\"";
static const char j_pump [] = "\"pump\":[%i,%i]";
static const char j_fan [] = "\"fan\":[%i,%i]";
static const char j_auto [] = "\"auto\":[%i,%i,%i]";
static const char j_temps [] = "\"temps\":[{\"l\":\"Water\",\"v\":%.1f},{\"l\":\"Air\",\"v\":%.1f},{\"l\":\"Other\",\"v\":%.1f}]";
static const char j_fan_max [] = "\"fanMax\":%i";
static const char j_led [] = "\"led\":%i";

static const char j_ui [] = "{\"myPanel\":{\"name\":\"Chili Pad\",\"ui\":{"
            "\"time_current\":{"
            "\"type\":\"datetime-local\","
            "\"label\":\"Time\","
            "\"cmd\":\"P0\""
            "},\"auto\":{"
            "\"type\":\"onStartStop\","
            "\"label\":\"Auto\","
            "\"cmd\":\"P3\""
            "},\"temps\":{"
            "\"type\":\"temps\","
            "\"cmd\":\"P11\""
            "},\"pump\":{"
            "\"type\":\"onDuty\","
            "\"label\":\"Pump\","
            "\"cmd\":\"P1\""
            "},\"fan\":{"
            "\"type\":\"onDuty\","
            "\"label\":\"Fan\","
            "\"cmd\":\"P2\""
            "},\"led\":{"
            "\"type\":\"boolean\","
            "\"label\":\"Light\","
            "\"cmd\":\"P5\""
            "},\"fanMax\":{"
            "\"type\":\"number\","
            "\"label\":\"Fan Max\","
            "\"cmd\":\"P4\""
            "}}}}\n";
/*
           
            */
//char arrays to format and send json to the web interface
static char status[400];
static char sendc[400];

//Analog to digital converter can be connected to the Tinybee sd card pins. (Should not be used at same time)
//Must tell the ads library to use pins 19 & 18 before starting the adc: Wire.setPins(19,18); scl -> tinybee SCK(18), sda -> tinybee MISO(19) 
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

//Timer library that creates up to 16 timers from a single hardware timer
ESP32Timer hTimer3(3); //The timer library will use hardware timer 3 which is also used for tone() or beepers by the tinybee
ESP32_ISRTimer ISR_Timer;

static int ISR_TIMER_READ_TEMPS;
static int ISR_TIMER_AUTO_ON;
static int ISR_TIMER_AUTO_OFF;

void CHILIPAD::alarm_attachOnButton(){
  attachInterrupt(PIN_ON_INTERRUPT,CHILIPAD::alarm_onButtonPushed,FALLING);
}

void CHILIPAD::alarm_onButtonPushed(){
  detachInterrupt(PIN_ON_INTERRUPT);//detach interrupt to avoid bouncing
  ISR_Timer.setTimeout(1000,CHILIPAD::alarm_attachOnButton);//reattach interrupt after 1 second
  pump_on = true;
  fan_on = true;
  CHILIPAD::writeOutputs();
  updateWebUi = true;
}
void CHILIPAD::alarm_low_water(){
  writePump();
}
void IRAM_ATTR alarm_readTemps(){
  read_temps = true;
}
void IRAM_ATTR alarm_autoOn(){
  if(S.auto_on){
    updateWebUi = true;
    pump_on = true;
    fan_on = true;
    CHILIPAD::writeOutputs();
    ISR_Timer.changeInterval(ISR_TIMER_AUTO_ON,86400000);//resquedule the timer for 24hrs (initial setup was unlikely 24 hr interval)
  }
}
void IRAM_ATTR alarm_autoOff(){
  if(S.auto_on){
    updateWebUi = true;
    pump_on = false;
    fan_on = false;
    CHILIPAD::writeOutputs();
    ISR_Timer.changeInterval(ISR_TIMER_AUTO_OFF,86400000);
  }
}


//ISR_timer ticks 
bool IRAM_ATTR hwTimerHandler(void * timerNo){
	ISR_Timer.run();
	return true;
}
CHILIPAD::CHILIPAD(){}

static int onS, dutyS, startS, stopS;//used in switch statment
void CHILIPAD::handleCommand(const int8_t c, String val){
    switch (c){
    case 0://current time
        if(val!=""){setTimeCurrent(val.c_str());}
        sprintf(status, j_time_current, getTimeCurrent());
        break;
    case 1://Pump
        sscanf(val.c_str(),"[%u,%u]",&onS, &dutyS);
        setPump(onS>0,dutyS);
        sprintf(status, j_pump,pump_on,S.duty_pump);
        break;
    case 2://Fan
        sscanf(val.c_str(),"[%u,%u]",&onS, &dutyS);
        setFan(onS>0,dutyS);
        sprintf(status, j_fan,fan_on,duty_fan);
        break;
    case 3://Auto mode a
        sscanf(val.c_str(),"[%u,%u,%u]",&onS, &startS, &stopS);
        setAuto(onS>0,startS,stopS);
        sprintf(status, j_auto,S.auto_on,S.time_start, S.time_stop);
        break;
    case 4://Fan Max Duty
        dutyS = val.toInt();
        S.duty_fan_max = max(0,min(100,dutyS));
        writeFan();
        sprintf(status, j_fan_max,S.duty_fan_max);
        break;
    case 5://LED
        led_on = val.toInt();
        writeLed();
        break;
    case 11://temps
        sprintf(status, j_temps, temp_water,temp_air,temp_other);
        break;
    case 113: 
        SERIAL_ECHO(j_ui);
        return;
    case 114://web interface requesting all settings
        snprintf(sendc,sizeof(sendc),"%s,%s,%s,%s,%s,%s,%s",j_time_current, j_pump, j_fan, j_auto, j_temps, j_led, j_fan_max);
        snprintf(status,sizeof(status),sendc,getTimeCurrent(),pump_on,S.duty_pump,fan_on,duty_fan,S.auto_on,S.time_start,S.time_stop,temp_water,temp_air,temp_other,led_on,S.duty_fan_max);
        break;
    default:
        SERIAL_ECHOLN("unknown chilipad command");
        return;
        break;
    }
    snprintf(sendc,sizeof(sendc),"%s%s%s",j_start, status, j_end);
    SERIAL_ECHOLN(sendc);
}

void CHILIPAD::setup(uint32_t cycle_period_ms){
    /*
    Wire.setPins(19,18);
    if(!ads.begin()){SERIAL_ECHOLN("i2c error");}
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0,false);
    */

    SET_OUTPUT(PIN_PUMP_DUTY);
    SET_OUTPUT(PIN_FAN_DUTY);
    SET_OUTPUT(PIN_LED);
    SET_INPUT_PULLUP(PIN_LEVEL_SWITCH);
    SET_INPUT_PULLUP(PIN_ON_INTERRUPT);
    SET_OUTPUT(PIN_12V_PUMP);

    setupClock();
    loadSettings();//get saved settings from eeprom
    if(!S.initialized){//the settings have never been saved (code loaded to device for the first time or eeprom was erased)
      S.initialized = true;
      S.auto_on = true;
      S.duty_pump = 50;
      S.duty_fan_max = 60;
      S.time_start = 73800;//8:30 pm
      S.time_stop = 32400;//60*60*9 (9:00am)
      S.temp_setPoint = 25;
      saveSettings();
    }
   
    if (hTimer3.attachInterruptInterval(1000,hwTimerHandler)){}//set hardware timer to run at 1ms (1000 us)
    //Up to 16 timers based on hTimer3. These timers tick length is determined by hTimer3 interrupt interval
    /*Note - reading an i2s analog to digital converter cannot be performed inside an ISR*/
    ISR_TIMER_READ_TEMPS = ISR_Timer.setInterval(2000,alarm_readTemps);
    ISR_TIMER_AUTO_ON = ISR_Timer.setInterval(SEC_PER_DAY*1000,alarm_autoOn);                  //auto on timer
    ISR_TIMER_AUTO_OFF = ISR_Timer.setInterval(SEC_PER_DAY*1000,alarm_autoOff);                //auto off timer
    
    setAutoTimers();//Auto on & off timers were initialized with arbitrary values above. Update them based on current time
    
    //setup chlorine pwm
    ledcSetup(pumpPWMChannel, 25000, 7);//1khz 7bits (0-128)
    ledcAttachPin(PIN_PUMP_DUTY, pumpPWMChannel);
    //ledcAttachPin(PIN_12V_PUMP,pumpPWMChannel);
    ledcSetup(fanPWMChannel, 25000, 7);//1khz 7bits (0-128)
    ledcAttachPin(PIN_FAN_DUTY, fanPWMChannel);
    
    
    //ledcSetup(ledPWMChannel,1000,7);
    //ledcAttachPin(PIN_12V_PUMP,ledPWMChannel);

    alarm_attachOnButton();//attach an interupt to the on button
    attachInterrupt(PIN_LEVEL_SWITCH,CHILIPAD::alarm_low_water,CHANGE);
    writeOutputs();
    CHILIPAD::handleCommand(114,"");//update ui
}

void CHILIPAD::loop(){
  if(updateWebUi){
    updateWebUi = false;
    CHILIPAD::handleCommand(114,"");//auto mode has turned the equipment on or off. Update the UI
  }
  if(read_temps){
    read_temps = false;
    temp_water = Temperature::temp_hotend[0].celsius;
    temp_air = Temperature::temp_hotend[1].celsius;
    temp_other = Temperature::temp_bed.celsius;
  }
  return;
  /*
  //very sluggish constantly reading adc
  if (!ads.conversionComplete()) {
    return;
  }
  a0 = 0.995f*a0 + 0.005*sq(ads.computeVolts(ads.getLastConversionResults())-2.5);
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, false);
  */
}

void CHILIPAD::setupClock(){
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
void CHILIPAD::timeSyncCallback(struct timeval *tv){
    setAutoTimers();
    CHILIPAD::handleCommand(0,"");//output time to serial 
}
void CHILIPAD::setAutoTimers(){
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

void CHILIPAD::setAuto(bool on, int start, int stop){
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

void CHILIPAD::setPump(bool on, int duty){
  pump_on = on;
  duty = max(0,min(100,duty));
  if(duty!=S.duty_pump){
    S.duty_pump = duty;
    saveSettings();//save to eeprom
  }
  writePump();
}
void CHILIPAD::setFan(bool on, int duty){
  fan_on = on;
  duty_fan = max(0,min(100,duty));
  writeFan();
}

void CHILIPAD::writeOutputs(){
  writePump();
  writeFan();
  writeLed();
}
void CHILIPAD::writePump(){
  int on = pump_on * (S.duty_pump > 0) * (digitalRead(PIN_LEVEL_SWITCH) < 1);
  OUT_WRITE(PIN_12V_PUMP,on);
  ledcWrite(pumpPWMChannel,S.duty_pump*127/100*on);
}
void CHILIPAD::writeFan(){
  int correctedDuty = duty_fan*128*S.duty_fan_max/10000*fan_on*pump_on;
  ledcWrite(fanPWMChannel,duty_fan*127*S.duty_fan_max/10000*fan_on*pump_on);
  //ledcWrite(fanPWMChannel,duty_fan*127/100*fan_on*pump_on);
}
void CHILIPAD::writeLed(){
  OUT_WRITE(PIN_LED,led_on);
}
void CHILIPAD::setTimeCurrent(const char *sDateTime){//2023-09-11T16:30:21
  struct tm ti;
  sscanf(sDateTime,"%d-%d-%dT%d:%d",&ti.tm_year,&ti.tm_mon,&ti.tm_mday,&ti.tm_hour,&ti.tm_min);
  ti.tm_year-=1900;
  ti.tm_mon-=1;//0-11
  time_t t = mktime(&ti);
  struct timeval now = { .tv_sec = t };
  settimeofday(&now, NULL);
  setAutoTimers();
}
char* CHILIPAD::getTimeCurrent(){
    time_t now;
    time(&now);
    struct tm lTime;
    localtime_r(&now,&lTime);
    static char dts[22];
    strftime(dts, sizeof(dts), "%Y-%m-%dT%X", &lTime);
    return dts;
}
void CHILIPAD::loadSettings(){
  EEPROM.begin(MARLIN_EEPROM_SIZE);
  EEPROM.get(EEPROM_SETTINGS_LOCATION,S);
  EEPROM.end();
}
void CHILIPAD::saveSettings(){
  if (!EEPROM.begin(MARLIN_EEPROM_SIZE)){
      SERIAL_ECHO("eeprom failed to start");
  }
  EEPROM.put(EEPROM_SETTINGS_LOCATION,S);
  EEPROM.end();
}

