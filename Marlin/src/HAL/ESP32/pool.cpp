#include "pool.h"
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
#define PIN_PUMP_POWER          EXP1_01_PIN
#define PIN_BOOSTER_POWER       EXP1_03_PIN
#define PIN_CHLORINE_DUTY_FWD   EXP1_04_PIN
#define PIN_CHLORINE_DUTY_REV   EXP1_05_PIN
#define PIN_ION_DUTY_FWD        EXP1_06_PIN
#define PIN_ION_DUTY_REV        EXP1_07_PIN
#define PIN_POWER_SUPPLY_ON     EXP1_08_PIN

#define PIN_ACID                HEATER_0_PIN
#define PIN_FLOC                HEATER_1_PIN
#define PIN_12V_PUMP            HEATER_BED_PIN
#define PIN_ON_INTERRUPT_A      Y_STOP_PIN
#define PIN_ON_INTERRUPT_B      Z_STOP_PIN

#define PIN_LED_PUMP            FAN_PIN
#define PIN_LED_BOOSTER         FAN1_PIN

struct PoolParameters{//26 bytes
  byte initialized;//1 byte = flag, used to determine if the eeprom has been initialized with pool settings yet
  byte auto_on_a;//1 byte = flag
  byte auto_on_b;
  int duty_chlorine;//4 bytes = int
  int duty_ion;//4 bytes = int
  int time_start_a;//4 bytes = int
  int time_stop_a;//4 bytes = int
  int time_start_b;//4 bytes = int
  int time_stop_b;//4 bytes = int
  int cycle_chlorine;//4 bytes = int
  int cycle_ion;//4 bytes = int
  byte acid_on;
  int duty_acid;
  byte floc_on;
  int duty_floc;
};
#define SEC_PER_DAY 86400
#define chlorinePWMChannel_fwd 0
#define chlorinePWMChannel_rev 1
#define ionPWMChannel_fwd 2
#define ionPWMChannel_rev 3

static PoolParameters S;
static volatile bool pump_on = false;
static volatile bool booster_on = false;
static volatile bool chlorine_on = true;
static volatile bool chlorine_rev = false;
static volatile bool ion_on = true;
static volatile bool ion_rev = false;
static volatile bool acid_on_cycle = false;
static volatile bool floc_on_cycle = false;
static volatile float a0 = 2.5f;
static volatile bool updateWebUi = false;

static const char j_start [] = "{\"myPanel\":{\"values\":{";
static const char j_end [] = "}}}\n";
static const char j_time_current [] = "\"time_current\":\"%s\"";
static const char j_pump_on [] = "\"pump\":%i";
static const char j_booster_on [] = "\"booster\":%i";
static const char j_auto_a [] = "\"auto_a\":[%i,%i,%i]";
static const char j_auto_b [] = "\"auto_b\":[%i,%i,%i]";
static const char j_chlorine [] = "\"chlorine\":[%i,%i]";
static const char j_ion [] = "\"ion\":[%i,%i]";
static const char j_acid [] = "\"acid\":[%i,%i]";
static const char j_floc [] = "\"floc\":[%i,%i]";
static const char j_cycle_chlorine [] = "\"cycle_chlorine\":%i";
static const char j_cycle_ion [] = "\"cycle_ion\":%i";
static const char j_temps [] = "\"temps\":[{\"l\":\"Outlet\",\"v\":%.1f},{\"l\":\"Inlet\",\"v\":%.1f},{\"l\":\"Ambient\",\"v\":%.1f}]";

static const char j_ui [] = "{\"myPanel\":{\"name\":\"Pool\",\"ui\":{"
            "\"time_current\":{"
            "\"type\":\"datetime-local\","
            "\"label\":\"Time\","
            "\"cmd\":\"P0\""
            "},\"auto_a\":{"
            "\"type\":\"onStartStop\","
            "\"label\":\"Auto\","
            "\"cmd\":\"P3\""
            "},\"auto_b\":{"
            "\"type\":\"onStartStop\","
            "\"label\":\"Auto 2\","
            "\"cmd\":\"P4\""
            "},\"temps\":{"
            "\"type\":\"temps\","
            "\"cmd\":\"P11\""
            "},\"pump\":{"
            "\"type\":\"boolean\","
            "\"label\":\"Pump\","
            "\"cmd\":\"P1\""
             "},\"booster\":{"
            "\"type\":\"boolean\","
            "\"label\":\"Booster\","
            "\"cmd\":\"P2\""
            "},\"chlorine\":{"
            "\"type\":\"onDuty\","
            "\"label\":\"Chlorinator\","
            "\"cmd\":\"P5\""
            "},\"ion\":{"
            "\"type\":\"onDuty\","
            "\"label\":\"Ionizer\","
            "\"cmd\":\"P6\""
            "},\"acid\":{"
            "\"type\":\"onDuty\","
            "\"label\":\"Acid Doser\","
            "\"cmd\":\"P7\""
            "},\"floc\":{"
            "\"type\":\"onDuty\","
            "\"label\":\"Floc Doser\","
            "\"cmd\":\"P8\""
            "},\"cycle_chlorine\":{"
            "\"type\":\"number\","
            "\"label\":\"Chlorine Period\","
            "\"append\":\"sec\","
            "\"cmd\":\"P9\""
            "},\"cycle_ion\":{"
            "\"type\":\"number\","
            "\"label\":\"Ion Period\","
            "\"append\":\"sec\","
            "\"cmd\":\"P10\""
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

static int ISR_TIMER_INVERT_CHLORINE;
static int ISR_TIMER_INVERT_ION;
static int ISR_TIMER_AUTO_ON_A;
static int ISR_TIMER_AUTO_OFF_A;
static int ISR_TIMER_AUTO_ON_B;
static int ISR_TIMER_AUTO_OFF_B;
static int ISR_TIMER_ACID_ONOFF;
static int ISR_TIMER_FLOC_ONOFF;

void POOL::alarm_attachOnButton_a(){
  attachInterrupt(PIN_ON_INTERRUPT_A,POOL::alarm_onButtonPushed_a,FALLING);
}
void POOL::alarm_onButtonPushed_a(){
  detachInterrupt(PIN_ON_INTERRUPT_A);//detach interrupt to avoid bouncing
  ISR_Timer.setTimeout(1000,POOL::alarm_attachOnButton_a);//reattach interrupt after 1 second
  POOL::pumpOn(!pump_on);
  updateWebUi = true;
}
void POOL::alarm_attachOnButton_b(){
  attachInterrupt(PIN_ON_INTERRUPT_B,POOL::alarm_onButtonPushed_b,FALLING);
}
void POOL::alarm_onButtonPushed_b(){
  detachInterrupt(PIN_ON_INTERRUPT_B);//detach interrupt to avoid bouncing
  ISR_Timer.setTimeout(1000,POOL::alarm_attachOnButton_b);//reattach interrupt after 1 second
  POOL::boosterOn(!booster_on);
  updateWebUi = true;
}
void IRAM_ATTR alarm_invertChlorine(){
  chlorine_rev = !chlorine_rev;
  POOL::writeChlorine();
}
void IRAM_ATTR alarm_invertIon(){
  ion_rev = !ion_rev;
  POOL::writeIon();
}
void IRAM_ATTR alarm_autoOn_a(){
  if(S.auto_on_a){
    updateWebUi = true;
    pump_on = true;
    chlorine_on = true;
    ion_on = true;
    POOL::writeOutputs();
    ISR_Timer.changeInterval(ISR_TIMER_AUTO_ON_A,86400000);//resquedule the timer for 24hrs (initial setup was unlikely 24 hr interval)
  }
}
void IRAM_ATTR alarm_autoOff_a(){
  if(S.auto_on_a){
    updateWebUi = true;
    pump_on = false;
    booster_on = false;
    chlorine_on = false;
    ion_on = false;
    POOL::writeOutputs();
    ISR_Timer.changeInterval(ISR_TIMER_AUTO_OFF_A,86400000);
  }
}
void IRAM_ATTR alarm_autoOn_b(){
  if(S.auto_on_b){
    updateWebUi = true;
    booster_on = true;
    POOL::writeOutputs();
    ISR_Timer.changeInterval(ISR_TIMER_AUTO_ON_B,86400000);//resquedule the timer for 24hrs (initial setup was unlikely 24 hr interval)
  }
}
void IRAM_ATTR alarm_autoOff_b(){
  if(S.auto_on_b){
    updateWebUi = true;
    booster_on = false;
    POOL::writeOutputs();
    ISR_Timer.changeInterval(ISR_TIMER_AUTO_OFF_B,86400000);
  }
}
void IRAM_ATTR alarm_acid(){
  if(acid_on_cycle){//turn acid off
    acid_on_cycle = false;
    POOL::writeAcid();
    ISR_Timer.changeInterval(ISR_TIMER_ACID_ONOFF,60000-S.duty_acid*50);//600,000ms = 10min
  }else{
    acid_on_cycle = true;
    POOL::writeAcid();
    ISR_Timer.changeInterval(ISR_TIMER_ACID_ONOFF,S.duty_acid*50);//600,000ms = 10min
  }
}
void IRAM_ATTR alarm_floc(){
  if(floc_on_cycle){//turn floc off
    floc_on_cycle = false;
    POOL::writeFloc();
    ISR_Timer.changeInterval(ISR_TIMER_FLOC_ONOFF,10000);//600,000ms = 10min
  }else{
    floc_on_cycle = true;
    POOL::writeFloc();
    ISR_Timer.changeInterval(ISR_TIMER_FLOC_ONOFF,S.duty_floc*100);//600,000ms = 10min
  }
}
//ISR_timer ticks 
bool IRAM_ATTR hwTimerHandler(void * timerNo){
	ISR_Timer.run();
	return true;
}
POOL::POOL(){}

static int onS, dutyS, startS, stopS;//used in switch statment
void POOL::handleCommand(const int8_t c, String val){
    switch (c){
    case 0://current time
        if(val!=""){setTimeCurrent(val.c_str());}
        sprintf(status, j_time_current, getTimeCurrent());
        break;
    case 1://Pump on off
        if(val!=""){pumpOn(val.toInt()>0);}
        //Turning off main pump turns off booster, chlorinator, ionizer, acid doser etc
        snprintf(sendc,sizeof(sendc),"%s,%s,%s,%s",j_pump_on, j_booster_on, j_chlorine, j_ion);
        snprintf(status,sizeof(status),sendc,pump_on,booster_on,chlorine_on,S.duty_chlorine,ion_on,S.duty_ion);
        break;
    case 2://Booster on off
        boosterOn(val.toInt()>0);
        sprintf(status, j_booster_on, booster_on);
        break;
    case 3://Auto mode a
        sscanf(val.c_str(),"[%u,%u,%u]",&onS, &startS, &stopS);
        setAuto_a(onS>0,startS,stopS);
        sprintf(status, j_auto_a,S.auto_on_a,S.time_start_a, S.time_stop_a);
        break;
    case 4://Auto mode b
        sscanf(val.c_str(),"[%u,%u,%u]",&onS, &startS, &stopS);
        setAuto_b(onS>0,startS,stopS);
        sprintf(status, j_auto_b,S.auto_on_b,S.time_start_b, S.time_stop_b);
        break;
    case 5://Chlorinator
        sscanf(val.c_str(),"[%u,%u]",&onS, &dutyS);
        setChlorine(onS>0,dutyS);
        sprintf(status, j_chlorine,chlorine_on,S.duty_chlorine);
        break;
    case 6://ionizer
        sscanf(val.c_str(),"[%u,%u]",&onS, &dutyS);
        setIon(onS>0,dutyS);
        sprintf(status, j_ion,ion_on,S.duty_ion);
        break;
    case 7://acid doser
        sscanf(val.c_str(),"[%u,%u]",&onS, &dutyS);
        setAcid(onS>0,dutyS);
        sprintf(status, j_acid, S.acid_on, S.duty_acid);
        break;
    case 8://floc doser
        sscanf(val.c_str(),"[%u,%u]",&onS, &dutyS);
        setFloc(onS>0,dutyS);
        sprintf(status, j_floc, S.floc_on, S.duty_floc);
    case 9://chlorine cycle
        cycleChlorine(val.toInt());
        sprintf(status, j_cycle_chlorine,S.cycle_chlorine);
        break;
    case 10://ion cycle
        cycleIon(val.toInt());
        sprintf(status, j_cycle_ion,S.cycle_ion);
        break;
    case 11://temps
        sprintf(status, j_temps, Temperature::temp_hotend[0].celsius,Temperature::temp_hotend[1].celsius,Temperature::temp_bed.celsius);
        break;
    case 113: 
        SERIAL_ECHO(j_ui);
        return;
    case 114://web interface requesting all settings
        snprintf(sendc,sizeof(sendc),"%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",j_time_current, j_pump_on, j_booster_on, j_auto_a, j_auto_b, j_chlorine, j_ion, j_acid, j_floc, j_cycle_chlorine, j_cycle_ion, j_temps);
        snprintf(status,sizeof(status),sendc,getTimeCurrent(),pump_on,booster_on,S.auto_on_a,S.time_start_a,S.time_stop_a,S.auto_on_b,S.time_start_b,S.time_stop_b,chlorine_on,S.duty_chlorine,ion_on,S.duty_ion,S.acid_on,S.duty_acid,S.floc_on,S.duty_floc,S.cycle_chlorine,S.cycle_ion,Temperature::temp_hotend[0].celsius,Temperature::temp_hotend[1].celsius,Temperature::temp_bed.celsius);
        break;
    default:
        SERIAL_ECHOLN("unknown pool command");
        return;
        break;
    }
    snprintf(sendc,sizeof(sendc),"%s%s%s",j_start, status, j_end);
    SERIAL_ECHOLN(sendc);
}

void POOL::setup(uint32_t cycle_period_ms){
    /*
    Wire.setPins(19,18);
    if(!ads.begin()){SERIAL_ECHOLN("i2c error");}
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0,false);
    */

    SET_OUTPUT(PIN_PUMP_POWER);
    SET_OUTPUT(PIN_BOOSTER_POWER);
    SET_OUTPUT(PIN_CHLORINE_DUTY_FWD);
    SET_OUTPUT(PIN_CHLORINE_DUTY_REV);
    SET_OUTPUT(PIN_ION_DUTY_FWD);
    SET_OUTPUT(PIN_ION_DUTY_REV);
    SET_OUTPUT(PIN_POWER_SUPPLY_ON);
    SET_OUTPUT(PIN_ACID);
    SET_OUTPUT(PIN_FLOC);
    SET_OUTPUT(PIN_12V_PUMP);
    SET_OUTPUT(PIN_LED_PUMP);
    SET_OUTPUT(PIN_LED_BOOSTER);
    SET_INPUT_PULLUP(PIN_ON_INTERRUPT_A);
    SET_INPUT_PULLUP(PIN_ON_INTERRUPT_B);

    setupClock();
    loadSettings();//get saved settings from eeprom
    if(!S.initialized){//the settings have never been saved (code loaded to device for the first time or eeprom was erased)
      S.initialized = true;
      S.auto_on_a = true;
      S.auto_on_b = false;
      S.duty_chlorine = 50;
      S.duty_ion = 50;
      S.time_start_a = 30600;//8:30 am
      S.time_stop_a = 55800;//60*15.5;//3:30 pm
      S.time_start_b = 30610;
      S.time_stop_b = 55790;
      S.cycle_chlorine = 600;//10 min (600 sec)
      S.cycle_ion = 600;//10 min (600 sec)
      S.acid_on = 0;
      S.duty_acid = 50;
      S.floc_on = 0;
      S.duty_floc = 50;
      saveSettings();
    }
   
    if (hTimer3.attachInterruptInterval(1000,hwTimerHandler)){}//set hardware timer to run at 1ms (1000 us)
    //Up to 16 timers based on hTimer3. These timers tick length is determined by hTimer3 interrupt interval
    /*Note - reading an i2s analog to digital converter cannot be performed inside an ISR*/
    ISR_TIMER_INVERT_CHLORINE = ISR_Timer.setInterval(S.cycle_chlorine*1000,alarm_invertChlorine); //chlorine reverse cycle timer
    ISR_TIMER_INVERT_ION = ISR_Timer.setInterval(S.cycle_ion*1000,alarm_invertIon);                //ionizer reverse cycle timer
    ISR_TIMER_AUTO_ON_A = ISR_Timer.setInterval(SEC_PER_DAY*1000,alarm_autoOn_a);                  //auto on timer
    ISR_TIMER_AUTO_OFF_A = ISR_Timer.setInterval(SEC_PER_DAY*1000,alarm_autoOff_a);                //auto off timer
    ISR_TIMER_AUTO_ON_B = ISR_Timer.setInterval(SEC_PER_DAY*1000,alarm_autoOn_b);                  //auto on timer
    ISR_TIMER_AUTO_OFF_B = ISR_Timer.setInterval(SEC_PER_DAY*1000,alarm_autoOff_b);                //auto off timer
    ISR_TIMER_ACID_ONOFF = ISR_Timer.setInterval(1000, alarm_acid);                                //acid dosing timer
    ISR_TIMER_FLOC_ONOFF = ISR_Timer.setInterval(1000, alarm_floc);                                //floc dosing timer
    setAutoTimers();//Auto on & off timers were initialized with arbitrary values above. Update them based on current time
    
    //setup chlorine pwm
    ledcSetup(chlorinePWMChannel_fwd, 1000, 7);//1khz 7bits (0-128)
    ledcAttachPin(PIN_CHLORINE_DUTY_FWD, chlorinePWMChannel_fwd);
    ledcSetup(chlorinePWMChannel_rev, 1000, 7);//1khz 7bits (0-128)
    ledcAttachPin(PIN_CHLORINE_DUTY_REV, chlorinePWMChannel_rev);
    
    //setup ion pwm
    ledcSetup(ionPWMChannel_fwd, 1000, 7);//1khz 7bits (0-128)
    ledcAttachPin(PIN_ION_DUTY_FWD, ionPWMChannel_fwd);
    ledcSetup(ionPWMChannel_rev, 1000, 7);//1khz 7bits (0-128)
    ledcAttachPin(PIN_ION_DUTY_REV, ionPWMChannel_rev);

    alarm_attachOnButton_a();//attach an interupt to the on button
    alarm_attachOnButton_b();//attach an interupt to the on button
    writeOutputs();
}

void POOL::loop(){
  if(updateWebUi){
    updateWebUi = false;
    POOL::handleCommand(1,"");//auto mode has turned the equipment on or off. Update the UI
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

void POOL::setupClock(){
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
void POOL::timeSyncCallback(struct timeval *tv){
    setAutoTimers();
    POOL::handleCommand(0,"");//output time to serial 
}
void POOL::setAutoTimers(){
    time_t now;
    time(&now);
    struct tm lTime;
    localtime_r(&now,&lTime);
    int sec = (lTime.tm_hour*3600 + lTime.tm_min*60 + lTime.tm_sec);
  
    int start = (S.time_start_a-sec+SEC_PER_DAY)%SEC_PER_DAY;
    int stop = (S.time_stop_a-sec+SEC_PER_DAY)%SEC_PER_DAY;
    ISR_Timer.changeInterval(ISR_TIMER_AUTO_ON_A,start*1000);
    ISR_Timer.changeInterval(ISR_TIMER_AUTO_OFF_A,stop*1000);

    start = (S.time_start_b-sec+SEC_PER_DAY)%SEC_PER_DAY;
    stop = (S.time_stop_b-sec+SEC_PER_DAY)%SEC_PER_DAY;
    ISR_Timer.changeInterval(ISR_TIMER_AUTO_ON_B,start*1000);
    ISR_Timer.changeInterval(ISR_TIMER_AUTO_OFF_B,stop*1000);
}

void POOL::setAuto_a(bool on, int start, int stop){
    bool changed = false;
    if(on != S.auto_on_a){
      S.auto_on_a = on;
      changed = true;
    }
    if(start != S.time_start_a){
      S.time_start_a = start;
      changed = true;
    }
    if(stop != S.time_stop_a){
      S.time_stop_a = stop;
      changed = true;
    }
    if(changed){
      setAutoTimers();
      saveSettings();//save to eeprom
    }
}
void POOL::setAuto_b(bool on, int start, int stop){
    bool changed = false;
    if(on != S.auto_on_b){
      S.auto_on_b = on;
      changed = true;
    }
    if(start != S.time_start_b){
      S.time_start_b = start;
      changed = true;
    }
    if(stop != S.time_stop_b){
      S.time_stop_b = stop;
      changed = true;
    }
    if(changed){
      setAutoTimers();
      saveSettings();//save to eeprom
    }
}
void POOL::pumpOn(bool on){
  pump_on = on;
  if(!pump_on){//pump must be on for chlorinator and ionizer to run
    booster_on = false;
    chlorine_on = false;
    ion_on = false;
  }
  writeOutputs();
}
void POOL::boosterOn(bool on){
  booster_on = on && pump_on;
  writeOutputs();
}
void POOL::setChlorine(bool on, int duty, bool write){
  chlorine_on = on && pump_on;
  if(duty!=S.duty_chlorine){
    S.duty_chlorine= duty;
    saveSettings();//save to eeprom
  }
  if(write){writeChlorine();}
}

void POOL::setIon(bool on, int duty, bool write){
  ion_on = on && pump_on;
  if(duty!=S.duty_ion){
    S.duty_ion = duty;
    saveSettings();//save to eeprom
  }
  if(write){writeIon();}
}
void POOL::setAcid(bool on, int duty){
  S.acid_on = on;
  if(duty!=S.duty_acid){
    S.duty_acid = duty;
    saveSettings();//save to eeprom
  }
  writeAcid();
}
void POOL::setFloc(bool on, int duty){
  S.floc_on = on;
  if(duty!=S.duty_floc){
    S.duty_floc = duty;
    saveSettings();//save to eeprom
  }
  writeFloc();
}
void POOL::writeOutputs(){
  WRITE(PIN_POWER_SUPPLY_ON,  pump_on);
  WRITE(PIN_PUMP_POWER,       pump_on);
  WRITE(PIN_LED_PUMP,         pump_on);
  WRITE(PIN_BOOSTER_POWER,    booster_on);
  WRITE(PIN_LED_BOOSTER,      booster_on);
  writeChlorine();
  writeIon();
  writeAcid();
  writeFloc();
}
void POOL::writeChlorine(){
  ledcWrite(chlorinePWMChannel_fwd,S.duty_chlorine*128/100*chlorine_on*!chlorine_rev*pump_on);
  ledcWrite(chlorinePWMChannel_rev,S.duty_chlorine*128/100*chlorine_on*chlorine_rev*pump_on);
}
void POOL::writeIon(){
  ledcWrite(ionPWMChannel_fwd,S.duty_ion*128/100*ion_on*!ion_rev*pump_on);
  ledcWrite(ionPWMChannel_rev,S.duty_ion*128/100*ion_on*ion_rev*pump_on);
}
void POOL::writeAcid(){
  WRITE(PIN_ACID,S.acid_on*acid_on_cycle*pump_on);
}
void POOL::writeFloc(){
  WRITE(PIN_FLOC,S.floc_on*floc_on_cycle*pump_on);
}
void POOL::cycleChlorine(int period_sec){
    if(period_sec!=S.cycle_chlorine){
      S.cycle_chlorine = period_sec;
      ISR_Timer.changeInterval(ISR_TIMER_INVERT_CHLORINE,period_sec*1000);
      saveSettings();//save to eeprom
    }
}
void POOL::cycleIon(int period_sec){
    if(period_sec!=S.cycle_ion){
      S.cycle_ion = period_sec;
      ISR_Timer.changeInterval(ISR_TIMER_INVERT_ION,period_sec*1000);
      saveSettings();//save to eeprom
    }
}
void POOL::setTimeCurrent(const char *sDateTime){//2023-09-11T16:30:21
  struct tm ti;
  sscanf(sDateTime,"%d-%d-%dT%d:%d",&ti.tm_year,&ti.tm_mon,&ti.tm_mday,&ti.tm_hour,&ti.tm_min);
  ti.tm_year-=1900;
  ti.tm_mon-=1;//0-11
  time_t t = mktime(&ti);
  struct timeval now = { .tv_sec = t };
  settimeofday(&now, NULL);
  setAutoTimers();
}
char* POOL::getTimeCurrent(){
    time_t now;
    time(&now);
    struct tm lTime;
    localtime_r(&now,&lTime);
    static char dts[22];
    strftime(dts, sizeof(dts), "%Y-%m-%dT%X", &lTime);
    return dts;
}
void POOL::loadSettings(){
  EEPROM.begin(MARLIN_EEPROM_SIZE);
  EEPROM.get(EEPROM_SETTINGS_LOCATION,S);
  EEPROM.end();
}
void POOL::saveSettings(){
  if (!EEPROM.begin(MARLIN_EEPROM_SIZE)){
      SERIAL_ECHO("eeprom failed to start");
  }
  EEPROM.put(EEPROM_SETTINGS_LOCATION,S);
  EEPROM.end();
}

