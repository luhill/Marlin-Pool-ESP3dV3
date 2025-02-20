#include "cement.h"
#include <driver/ledc.h> //used for pwm
#include <EEPROM.h>
#include <time.h>
#include <sys/time.h>
#include "esp_sntp.h"
#include <FastLED.h>
#include "../../core/serial.h"                             
#include "../../core/macros.h"
#include "../../pins/esp32/pins_MKS_TINYBEE.h"
#include "../../gcode/gcode.h"
#include "../../module/motion.h" //access to destination and prepare_line_to_destination
#include "../../module/planner.h" //access to buffer_
//#include "../../module/motion.h" //used to check position of the extruder

//#define _TIMERINTERRUPT_LOGLEVEL_     4
#include "ESP32TimerInterrupt.h"
#include "ArduinoJson.h"
#define EEPROM_SETTINGS_LOCATION (MARLIN_EEPROM_SIZE-400)

FASTLED_USING_NAMESPACE

#define PIN_WATERFALL_DATA    13
#define PIN_FLOODS 32
//#define CLK_PIN   4
#define LED_TYPE    WS2811Controller800Khz
#define COLOR_ORDER RGB
#define NUM_LEDS    20
CRGB leds[NUM_LEDS];

volatile bool onA = true;
volatile bool onB = true;
//volatile uint8_t dutyA = 255;
#define FRAMES_PER_SECOND  120

char c[] = "0xFFFF00";
//uint8_t effect = 1;

void addGlitter(fract8 chanceOfGlitter);
void solid();
void rainbow();
void nextPattern();
void confetti();
void sinelon();
void rainbowWithGlitter();
void juggle();
void bpm();

typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { solid, rainbow, confetti, sinelon, juggle, bpm };

struct Settings{//26 bytes
  byte initialized;//1 byte = flag, used to determine if the eeprom has been initialized with our settings yet
  byte auto_on;//1 byte = flag
  int time_start;//4 bytes = int
  int time_stop;//4 bytes = int
  uint8_t effect;
  uint8_t dutyA;
  uint8_t dutyB;
  CRGB color;
  //char color[9];
};
static Settings S;

#define SEC_PER_DAY 86400


static volatile bool updateWebUi = false;


//char arrays to format and send json to the web interface
static char status[1000];//make sure marlin MAX_CMD_SIZE and S2S_RXBUFFERSIZE (in serial2socket.h) are sized correctly
//static char serial2_msg[1000];//serial2 used to allow two tinybees to communicate

//Timer library that creates up to 16 timers from a single hardware timer
ESP32Timer hTimer3(3); //The timer library will use hardware timer 3 which is also used for tone() or beepers by the tinybee
ESP32_ISRTimer ISR_Timer;

static int ISR_TIMER_AUTO_ON;
static int ISR_TIMER_AUTO_OFF;
static int ISR_TIMER_DOSER;


//static abce_long_t doseTargets;
//static abce_pos_t pos_0;
JsonDocument doc, cmd;

void IRAM_ATTR alarm_autoOn(){
  if(S.auto_on){
    updateWebUi = true;
    onA = true;
    onB = true;
    //todo: task when on triggered
    CEMENT::writeOutputs();
    ISR_Timer.changeInterval(ISR_TIMER_AUTO_ON,86400000);//resquedule the timer for 24hrs (initial setup was unlikely 24 hr interval)
  }
}
void IRAM_ATTR alarm_autoOff(){
  if(S.auto_on){
    updateWebUi = true;
    onA = false;
    onB = false;
    //todo: task when off triggered
    CEMENT::writeOutputs();
    ISR_Timer.changeInterval(ISR_TIMER_AUTO_OFF,86400000);
  }
}



/*Called from gcode when an extrude command is seen
if extrude_on = true the additives and additional extruders are updated to provide appropriate dosage for the extrusion position
*/
void CEMENT::e_destination_changed(bool doseInline){

}

//ISR_timer ticks 
bool IRAM_ATTR hwTimerHandler(void * timerNo){
	ISR_Timer.run();
	return true;
}
CEMENT::CEMENT(){}

static int j;
static float v;
void CEMENT::handleCommand(String val){
  //return on invalid command
  DeserializationError error = deserializeJson(cmd, val);
  if(error){SERIAL_ECHOLN("Invalid json command");return;}
  
  doc.to<JsonObject>();//clear document
  JsonObject myPanel = doc[F("myPanel")].to<JsonObject>();  //doc.createNestedObject(F("myPanel"));
  JsonObject send = myPanel[F("values")].to<JsonObject>(); //myPanel.createNestedObject(F("values"));
  JsonVariant value;
  

  value = cmd[F("fetch_ui")];/*both*/
  if (!value.isNull()){
    buildUi();
    reply();
  }
  value = cmd[F("fetch_values")];/*both*/
  if (!value.isNull()){
    buildValues();
    reply();
    return;
  }

  value = cmd[F("auto")];/*both*/
  if (!value.isNull()){
    setAuto(value[0],value[1],value[2]);
    send[F("auto")][0] = S.auto_on;
    send[F("auto")][1] = S.time_start;
    send[F("auto")][2] = S.time_stop;
    reply();
    return;
  }

  value = cmd[F("waterfall")];/*both*/
  if (!value.isNull()){
    onA = value[0]>0;
    S.dutyA = (uint8_t)value[1].as<unsigned int>();
    FastLED.setBrightness(onA*S.dutyA);
    send[F("waterfall")][0] = onA;
    send[F("waterfall")][1] = S.dutyA;
    saveSettings();
    reply();
    return;
  }

  value = cmd[F("color")];/*both*/
  if (!value.isNull()){
    strncpy(&c[1], value.as<String>().c_str(),7);//html color picker sends "#RRGGBB"; Need to keep 0 in c[0] and change # to x
    c[0]='0';
    c[1]='x';
    S.color = CRGB(strtol(c, NULL, 0));
    solid();
    S.effect = 0;
    //effect = 0;//switch to solid color mode
    send[F("color")] = value;
    send[F("effect")] = S.effect; //upate web ui to show solid
    saveSettings();
    reply();
    return;
  }
  value = cmd[F("effect")];/*both*/
  if (!value.isNull()){
    S.effect = (uint8_t)value.as<unsigned int>();
    gPatterns[S.effect]();
    send[F("effect")] = S.effect;
    saveSettings();
    reply();
    return;
  }
  value = cmd[F("floods")];/*both*/
  if (!value.isNull()){
    onB = value[0]>0;
    S.dutyB = (uint8_t)value[1].as<unsigned int>();
    S.dutyB = min(S.dutyB,(uint8_t)200);
    analogWrite(PIN_FLOODS,S.dutyB*onB);
    //FastLED.setBrightness(onB*S.dutyB);
    send[F("floods")][0] = onB;
    send[F("floods")][1] = S.dutyB;
    saveSettings();
    reply();
    return;
  }

  SERIAL_ECHOLN("Cement cmd not recognised");
}

void CEMENT::reply(){
  serializeJson(doc,status);
  SERIAL_ECHOLN(status);
}
void CEMENT::endPrint(){
}

void CEMENT::buildUi(){
  doc.to<JsonObject>();//clear document
  JsonObject myPanel = myPanel = doc[F("myPanel")].to<JsonObject>();
  myPanel[F("name")] = F("Pool Lights");
  JsonObject ui = myPanel[F("ui")].to<JsonObject>();
  JsonObject item;
  
  
  //Date and time
  item = ui[F("time")].to<JsonObject>();
  item[F("type")] = F("datetime-local");
  item[F("label")] = F("Time");
  

  //Pump command
  //item = ui[F("pump")].to<JsonObject>();
  //item[F("type")] = F("extruder");

  
  //Ability to set an auto start / stop timer
  item = ui[F("auto")].to<JsonObject>();
  item[F("type")] = F("onStartStop");
  item[F("label")] = F("Auto");
  

  /*
  item = ui[F("mix")].to<JsonObject>();
  item[F("type")] = F("table");
  item[F("label")] = F("Mix");
  for(int i = 0; i < NUM_ADDITIVES; i++){
    item[F("rows")][i] = additives[i].name;//
  }
  item[F("cols")] = serialized(F("[\"Conc (g/L)\",\"Flow (g/s)\",\"Dose (g)\"]"));//table column headers
  */
  //
  item = ui[F("waterfall")].to<JsonObject>();
  item[F("type")] = F("onDuty");
  item[F("label")] = F("Waterfall");

  item = ui[F("color")].to<JsonObject>();
  item[F("type")] = F("color");
  item[F("label")] = F("Color");

  item = ui[F("effect")].to<JsonObject>();
  item[F("type")] = F("select");
  item[F("label")] = F("Effect");
  item[F("options")][0][F("value")] = 0;
  item[F("options")][0][F("label")] = F("Solid");
  item[F("options")][1][F("value")] = 1;
  item[F("options")][1][F("label")] = F("Rainbow");
  item[F("options")][2][F("value")] = 2;
  item[F("options")][2][F("label")] = F("Confetti");
  item[F("options")][3][F("value")] = 3;
  item[F("options")][3][F("label")] = F("Sinelon");
  item[F("options")][4][F("value")] = 4;
  item[F("options")][4][F("label")] = F("Juggle");
  item[F("options")][5][F("value")] = 5;
  item[F("options")][5][F("label")] = F("BPM");

  item = ui[F("floods")].to<JsonObject>();
  item[F("type")] = F("onDuty");
  item[F("label")] = F("Flood Lights");

  //solid, rainbow, confetti, sinelon, juggle, bpm
  /*item = ui[F("freq")].to<JsonObject>();
  item[F("type")] = F("number");
  item[F("label")] = F("PWM Frequency");
  item[F("append")] = F("hz");

  item = ui[F("aux1")].to<JsonObject>();
  item[F("type")] = F("onDuty");
  item[F("label")] = F("Aux1");

  item = ui[F("setPin")].to<JsonObject>();
  item[F("type")] = F("onDuty");
  item[F("label")] = F("Set Pin");
  */
}
void CEMENT::buildValues(){
  doc.to<JsonObject>();//clear document
  
  JsonObject myPanel = doc[F("myPanel")].to<JsonObject>();
  JsonObject values = myPanel[F("values")].to<JsonObject>();//myPanel.createNestedObject(F("values"));
  values[F("time")] = getTime();
  values[F("auto")][0] = S.auto_on;
  values[F("auto")][1] = S.time_start;
  values[F("auto")][2] = S.time_stop;
  values[F("waterfall")][0] = onA;
  values[F("waterfall")][1] = S.dutyA;
  //values[F("color")] = std::to_string((uint32_t)S.color);
  sprintf(c,"#%02X%02X%02X",S.color.r,S.color.g,S.color.b);
  values[F("color")] = c;
  values[F("effect")] = S.effect;
  values[F("floods")][0] = onB;
  values[F("floods")][1] = S.dutyB;
}
void CEMENT::setup(){
    //Set Pin states
    setupClock();
    loadSettings();//get saved settings from eeprom
    if(!S.initialized){//the settings have never been saved (code loaded to device for the first time or eeprom was erased)
      S.initialized = true;
      S.auto_on = false;
      S.time_start = 64800;//6:00 pm
      S.time_stop = 75600;//8:00 pm
      S.color = 0xFFFF00;
      S.dutyA = 64;
      S.dutyB = 64;
      S.effect = 1;
      saveSettings();
    }
   
    if (hTimer3.attachInterruptInterval(1000,hwTimerHandler)){}//set hardware timer to run at 1ms (1000 us)
    //Up to 16 timers based on hTimer3. These timers tick length is determined by hTimer3 interrupt interval
    /*Note - reading an i2s analog to digital converter cannot be performed inside an ISR*/
    ISR_TIMER_AUTO_ON = ISR_Timer.setInterval(SEC_PER_DAY*1000,alarm_autoOn);                      //auto on timer
    ISR_TIMER_AUTO_OFF = ISR_Timer.setInterval(SEC_PER_DAY*1000,alarm_autoOff);                    //auto off timer
    setAutoTimers();//Auto on & off timers were initialized with arbitrary values above. Update them based on current time
    /*
    ledcSetup(blenderPWMChannel, pwm_freq, 7);//3khz 7bits (0-128)
    ledcAttachPin(PIN_BLENDER, blenderPWMChannel);

    ledcSetup(auxPWMChannel, pwm_freq, 7);//3khz 7bits (0-128)
    ledcAttachPin(PIN_AUX, auxPWMChannel);

    writeOutputs();
    */
   pinMode(PIN_WATERFALL_DATA,OUTPUT);
   pinMode(PIN_FLOODS,OUTPUT);
  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,PIN_WATERFALL_DATA,COLOR_ORDER>(leds, NUM_LEDS);//.setCorrection(TypicalLEDStrip);
  //FastLED.addLeds<LED_TYPE,DATA_PIN,CLK_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  gPatterns[S.effect]();
  // set master brightness control
  //FastLED.setBrightness(onA*S.dutyA);
  writeOutputs();
}


//SimplePatternList gPatterns = {solid};

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns

bool CEMENT::isBusy(){
  return false;
}
void CEMENT::loop(){
  if(updateWebUi){
    updateWebUi = false;
    CEMENT::handleCommand(F("{\"fetch_values\":1}"));//auto mode has turned the equipment on or off. Update the UI
    //CEMENT::handleCommand(114,"");
  }
  // Call the current pattern function once, updating the 'leds' array
  gPatterns[S.effect]();

  // send the 'leds' array out to the actual LED strip
  FastLED.show();  
  // insert a delay to keep the framerate modest
  FastLED.delay(1000/FRAMES_PER_SECOND); 

  // do some periodic updates
  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
  //EVERY_N_SECONDS( 10 ) { nextPattern(); } // change patterns periodically
  return;
}
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))
void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}
void solid(){
  fill_solid(leds,NUM_LEDS,S.color);
}
void rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void rainbowWithGlitter() 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( fract8 chanceOfGlitter) 
{
  if( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void confetti() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16( 13, 0, NUM_LEDS-1 );
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  uint8_t dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16( i+7, 0, NUM_LEDS-1 )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
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
    updateWebUi = true;
    //CEMENT::handleCommand(F("{\"time\":\"\"}"));//output time to serial 
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
  //to use pwm on i2s pins (pin# > 127) use hal.set_pwm_frequency(PIN_NAME,freq); hal.set_pwm_duty(HEATER_BED_PIN,duty); as defined in ESP32/HAL.h
  FastLED.setBrightness(onA*S.dutyA);
  analogWrite(PIN_FLOODS,onB*S.dutyB);
}

void CEMENT::setTime(const char *sDateTime){//2023-09-11T16:30:21
  struct tm ti;
  sscanf(sDateTime,"%d-%d-%dT%d:%d",&ti.tm_year,&ti.tm_mon,&ti.tm_mday,&ti.tm_hour,&ti.tm_min);
  ti.tm_year-=1900;
  ti.tm_mon-=1;//0-11
  time_t t = mktime(&ti);
  struct timeval now = { .tv_sec = t };
  settimeofday(&now, NULL);
  setAutoTimers();
}
char* CEMENT::getTime(){
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

