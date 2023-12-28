#include "cement.h"
#include <driver/ledc.h> //used for pwm
#include <EEPROM.h>
#include <time.h>
#include <sys/time.h>
#include "esp_sntp.h"
#include "../../core/serial.h"                             
#include "../../core/macros.h"
#include "../../pins/esp32/pins_MKS_TINYBEE.h"
//#include "../../module/motion.h" //used to check position of the extruder


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
/*------------Cat6 Cable 1------------------
pin     extruder 1          extruder 2
    en: green               blue
  step: orange              brown
   dir: striped green       striped blue
   gnd: striped orange      striped brown
*/

/*------------Cat6 Cable 2------------------*/
#define PIN_AUX  EXP1_03_PIN //striped green
#define PIN_WATER     EXP1_04_PIN //green
#define PIN_DOSE_A    EXP1_05_PIN //striped blue
#define PIN_DOSE_B    EXP1_06_PIN //blue
#define PIN_DOSE_C    EXP1_07_PIN //striped brown wire / blue screw terminal
#define PIN_BLENDER   EXP1_08_PIN //orange
//ground is EXP1 pin 9 to striped orange
//5v is EXP1 pin 10 to solid brown

struct Settings{//26 bytes
  byte initialized;//1 byte = flag, used to determine if the eeprom has been initialized with our settings yet
  byte auto_on;//1 byte = flag
  int time_start;//4 bytes = int
  int time_stop;//4 bytes = int
};
static Settings S;

struct Addative{
  float ratio;//how much to add per mm of extrusion
  float flow; //how many ml/sec is dosed when turned on
  uint32_t position; //total duration in ms that addative has been dosed
  uint32_t targetPosition; 
  bool on;
  uint8_t pin;
};

static const uint8_t NUM_ADDATIVES = 2;
static volatile Addative addatives[NUM_ADDATIVES];

#define SEC_PER_DAY 86400

#define blenderPWMChannel 0
#define auxPWMChannel 1

static const int DOSE_INTERVAL_MS = 500;
static volatile bool updateWebUi = false;
static volatile bool on = false;
static volatile float_t e = 0.0f;
static const char j_start [] = "{\"myPanel\":{\"values\":{";
static const char j_end [] = "}}}\n";
static const char j_time_current [] = "\"time_current\":\"%s\"";
static const char j_extrude_on [] = "\"extrude\":%i";
static const char j_auto  [] = "\"auto\":[%i,%i,%i]";
static const char j_blend [] = "\"blend\":[%i,%i]";
static const char j_water [] = "\"water\":%i";
static const char j_doseA [] = "\"doseA\":%i";
static const char j_doseB [] = "\"doseB\":%i";
static const char j_doseC [] = "\"doseC\":%i";
static const char j_aux   [] = "\"aux\":[%i,%i]";

static const char j_ui [] = "{\"myPanel\":{\"name\":\"Cement Printer\",\"ui\":{"
            "\"time_current\":{"
            "\"type\":\"datetime-local\","
            "\"label\":\"Time\","
            "\"cmd\":\"P0\""
            "},\"extrude\":{"
            "\"type\":\"boolean\","
            "\"label\":\"Extrude\","
            "\"cmd\":\"P1\""
            "},\"auto\":{"
            "\"type\":\"onStartStop\","
            "\"label\":\"Auto\","
            "\"cmd\":\"P3\""
            "},\"blend\":{"
            "\"type\":\"onDuty\","
            "\"label\":\"Blender\","
            "\"cmd\":\"P4\""
            "},\"water\":{"
            "\"type\":\"boolean\","
            "\"label\":\"Water\","
            "\"cmd\":\"P5\""
            "},\"doseA\":{"
            "\"type\":\"boolean\","
            "\"label\":\"Dose A\","
            "\"cmd\":\"P6\""
            "},\"doseB\":{"
            "\"type\":\"boolean\","
            "\"label\":\"Dose B\","
            "\"cmd\":\"P7\""
            "},\"doseC\":{"
            "\"type\":\"boolean\","
            "\"label\":\"Dose C\","
            "\"cmd\":\"P8\""
            "},\"aux\":{"
            "\"type\":\"onDuty\","
            "\"label\":\"Auxilary\","
            "\"cmd\":\"P9\""
            "}}}}\n";
//char arrays to format and send json to the web interface
static char status[800];
static char sendc[800];

//Timer library that creates up to 16 timers from a single hardware timer
ESP32Timer hTimer3(3); //The timer library will use hardware timer 3 which is also used for tone() or beepers by the tinybee
ESP32_ISRTimer ISR_Timer;

static int ISR_TIMER_AUTO_ON;
static int ISR_TIMER_AUTO_OFF;
static int ISR_TIMER_DOSER;
static int ISR_TIMER_WATER_OFF;
static int ISR_TIMER_DOSE_A_OFF;
static int ISR_TIMER_DOSE_B_OFF;
static int ISR_TIMER_DOSE_C_OFF;

static volatile bool extrude_on = false;
static volatile bool blender_on = false;
static volatile bool water_on = false;
static volatile bool doseA_on = false;
static volatile bool doseB_on = false;
static volatile bool doseC_on = false;
static volatile bool aux_on = false;
static volatile int duty_blender = 10;
static volatile int duty_aux = 10;

void IRAM_ATTR alarm_autoOn(){
  if(S.auto_on){
    updateWebUi = true;
    //todo: task when on triggered
    CEMENT::writeOutputs();
    ISR_Timer.changeInterval(ISR_TIMER_AUTO_ON,86400000);//resquedule the timer for 24hrs (initial setup was unlikely 24 hr interval)
  }
}
void IRAM_ATTR alarm_autoOff(){
  if(S.auto_on){
    updateWebUi = true;
    //todo: task when off triggered
    CEMENT::writeOutputs();
    ISR_Timer.changeInterval(ISR_TIMER_AUTO_OFF,86400000);
  }
}
void IRAM_ATTR alarm_water_off(){
  water_on = false;
  updateWebUi = true;
  CEMENT::writeWater();
}
void IRAM_ATTR alarm_doseA_off(){
  doseA_on = false;
  updateWebUi = true;
  CEMENT::writeDoseA();
}
void IRAM_ATTR alarm_doseB_off(){
  doseB_on = false;
  updateWebUi = true;
  CEMENT::writeDoseB();
}
void IRAM_ATTR alarm_doseC_off(){
  doseC_on = false;
  updateWebUi = true;
  CEMENT::writeDoseC();
}
//Periodically check the extrusion position and dose chemicals if required
void IRAM_ATTR alarm_dose(){// floating point calcs not allowed in ISR (hence doubles)
  if(!extrude_on) return;
  for(int i = 0; i < NUM_ADDATIVES; i++){
    if(addatives[i].on){
      addatives[i].position += DOSE_INTERVAL_MS;
    }
    addatives[i].on = addatives[i].position < addatives[i].targetPosition;
    WRITE(addatives[i].pin,addatives[i].on);
  }
}
void CEMENT::extrude(float_t ex){
  //e = ex;
  for(int i = 0; i < NUM_ADDATIVES; i++){
    addatives[i].targetPosition = (int32_t)(ex * addatives[i].ratio / addatives[i].flow*1000);
  }
}
//ISR_timer ticks 
bool IRAM_ATTR hwTimerHandler(void * timerNo){
	ISR_Timer.run();
	return true;
}
CEMENT::CEMENT(){}

static int onS, dutyS, startS, stopS;//used in switch statment
void CEMENT::handleCommand(const int8_t c, String val){
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
    case 1://Extrude matching on off
        extrude_on = val.toInt()>0;
        sprintf(status, j_extrude_on, extrude_on);
        break;
    case 3://Auot timer
        sscanf(val.c_str(),"[%u,%u,%u]",&onS, &startS, &stopS);
        setAuto(onS>0,startS,stopS);
        sprintf(status, j_auto,S.auto_on,S.time_start, S.time_stop);
        break;
    case 4://blender
        sscanf(val.c_str(),"[%u,%u]",&onS, &dutyS);
        setBlender(onS>0,dutyS);
        sprintf(status, j_blend,blender_on,duty_blender);
        break;
    case 5://water
        waterOn(val.toInt()>0);
        sprintf(status, j_water, water_on);
        break;
    case 6://dose A
        doseAOn(val.toInt()>0);
        sprintf(status, j_doseA, doseA_on);
        break;
    case 7://dose B
        doseBOn(val.toInt()>0);
        sprintf(status, j_doseB, doseB_on);
        break;
    case 8://dose C
        doseCOn(val.toInt()>0);
        sprintf(status, j_doseC, doseC_on);
        break;
    case 9://dose D
        sscanf(val.c_str(),"[%u,%u]",&onS, &dutyS);
        setAux(onS>0,dutyS);
        sprintf(status, j_aux,aux_on,duty_aux);
        break;
    case 113://update ui
        SERIAL_ECHO(j_ui);
        return;
    case 114://web interface requesting all settings
        snprintf(sendc,sizeof(sendc),"%s,%s,%s,%s,%s,%s,%s,%s,%s",j_time_current, j_auto, j_extrude_on,j_blend,j_water,j_doseA,j_doseB,j_doseC,j_aux);
        snprintf(status,sizeof(status),sendc,getTimeCurrent(),S.auto_on,S.time_start,S.time_stop,extrude_on,blender_on,duty_blender,water_on,doseA_on,doseB_on,doseC_on,aux_on,duty_aux);
        break;
    default:
        SERIAL_ECHOLN("unknown cement command");
        return;
        break;
    }
    snprintf(sendc,sizeof(sendc),"%s%s%s",j_start, status, j_end);
    SERIAL_ECHOLN(sendc);
}
void CEMENT::initializeAddatives(){
  addatives[0].ratio = 1.0f;
  addatives[0].flow = 1.0f;
  addatives[0].position = 0;
  addatives[0].targetPosition = 0;
  addatives[0].on = false;
  addatives[0].pin = PIN_DOSE_A; 

  addatives[1].ratio = 0.5f;
  addatives[1].flow = 1.0f;
  addatives[1].position = 0;
  addatives[1].targetPosition = 0;
  addatives[1].on = false;
  addatives[1].pin = PIN_DOSE_B;
}
void CEMENT::setup(){
    //Set Pin states
    SET_OUTPUT(PIN_BLENDER);
    SET_OUTPUT(PIN_WATER);
    SET_OUTPUT(PIN_DOSE_A);
    SET_OUTPUT(PIN_DOSE_B);
    SET_OUTPUT(PIN_DOSE_C);
    SET_OUTPUT(PIN_AUX);

    initializeAddatives();
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
    ISR_TIMER_DOSER = ISR_Timer.setInterval(DOSE_INTERVAL_MS, alarm_dose);
    setAutoTimers();//Auto on & off timers were initialized with arbitrary values above. Update them based on current time
    
    ledcSetup(blenderPWMChannel, 3000, 7);//3khz 7bits (0-128)
    ledcAttachPin(PIN_BLENDER, blenderPWMChannel);

    ledcSetup(auxPWMChannel, 3000, 7);//3khz 7bits (0-128)
    ledcAttachPin(PIN_AUX, auxPWMChannel);

    writeOutputs();
}

void CEMENT::loop(){
  if(updateWebUi){
    updateWebUi = false;
    CEMENT::handleCommand(114,"");//auto mode has turned the equipment on or off. Update the UI
  }
  return;
}

void CEMENT::setupClock(){
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
void CEMENT::timeSyncCallback(struct timeval *tv){
    setAutoTimers();
    CEMENT::handleCommand(0,"");//output time to serial 
}
void CEMENT::setAutoTimers(){
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

void CEMENT::setAuto(bool on, int start, int stop){
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

void CEMENT::writeOutputs(){
  //Do not use 'digitalWrite' use macros defined in ESP32/fastio.h ('WRITE') as some mks tinybee pins are virtual shift register pins
  //WRITE(PIN_NAME,HIGH)
  writeBlender();
  writeWater();
  writeDoseA();
  writeDoseB();
  writeDoseC();
  writeAux();
}
void CEMENT::setBlender(bool on, int duty){
  blender_on = on;
  duty_blender = duty;
  writeBlender();
}
void CEMENT::setAux(bool on, int duty){
  aux_on = on;
  duty_aux = duty;
  writeAux();
}
void CEMENT::waterOn(bool on){
  water_on = on;
  if(water_on){
    ISR_TIMER_WATER_OFF = ISR_Timer.setTimeout(3*1000, alarm_water_off);//3 sec on
    writeWater();
  }else{
    ISR_Timer.deleteTimer(ISR_TIMER_WATER_OFF);
    alarm_water_off();
  }
}
void CEMENT::doseAOn(bool on){
  doseA_on = on;
  if(doseA_on){
    ISR_TIMER_DOSE_A_OFF = ISR_Timer.setTimeout(3*1000, alarm_doseA_off);//3 sec on
    writeDoseA();
  }else{
    ISR_Timer.deleteTimer(ISR_TIMER_DOSE_A_OFF);
    alarm_doseA_off();
  }
}
void CEMENT::doseBOn(bool on){
  doseB_on = on;
  if(doseB_on){
    ISR_TIMER_DOSE_B_OFF = ISR_Timer.setTimeout(3*1000, alarm_doseB_off);//3 sec on
    writeDoseB();
  }else{
    ISR_Timer.deleteTimer(ISR_TIMER_DOSE_B_OFF);
    alarm_doseB_off();
  }
}
void CEMENT::doseCOn(bool on){
  doseC_on = on;
  if(doseC_on){
    ISR_TIMER_DOSE_C_OFF = ISR_Timer.setTimeout(3*1000, alarm_doseC_off);//3 sec on
    writeDoseC();
  }else{
    ISR_Timer.deleteTimer(ISR_TIMER_DOSE_C_OFF);
    alarm_doseC_off();
  }
}
void CEMENT::writeBlender(){
  ledcWrite(blenderPWMChannel,duty_blender*128/100*blender_on);
}
void CEMENT::writeWater(){
  WRITE(PIN_WATER, water_on);
}
void CEMENT::writeDoseA(){
  WRITE(PIN_DOSE_A, doseA_on);
}
void CEMENT::writeDoseB(){
  WRITE(PIN_DOSE_B, doseB_on);
}
void CEMENT::writeDoseC(){
  WRITE(PIN_DOSE_C, doseC_on);
}
void CEMENT::writeAux(){
  ledcWrite(auxPWMChannel,duty_aux*128/100*aux_on);
}
void CEMENT::setTimeCurrent(const char *sDateTime){//2023-09-11T16:30:21
  struct tm ti;
  sscanf(sDateTime,"%d-%d-%dT%d:%d",&ti.tm_year,&ti.tm_mon,&ti.tm_mday,&ti.tm_hour,&ti.tm_min);
  ti.tm_year-=1900;
  ti.tm_mon-=1;//0-11
  time_t t = mktime(&ti);
  struct timeval now = { .tv_sec = t };
  settimeofday(&now, NULL);
  setAutoTimers();
}
char* CEMENT::getTimeCurrent(){
    time_t now;
    time(&now);
    struct tm lTime;
    localtime_r(&now,&lTime);
    static char dts[22];
    strftime(dts, sizeof(dts), "%Y-%m-%dT%X", &lTime);
    return dts;
}
void CEMENT::loadSettings(){
  EEPROM.begin(MARLIN_EEPROM_SIZE);
  EEPROM.get(EEPROM_SETTINGS_LOCATION,S);
  EEPROM.end();
}
void CEMENT::saveSettings(){
  if (!EEPROM.begin(MARLIN_EEPROM_SIZE)){
      SERIAL_ECHO("eeprom failed to start");
  }
  EEPROM.put(EEPROM_SETTINGS_LOCATION,S);
  EEPROM.end();
}

