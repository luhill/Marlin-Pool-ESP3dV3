#include "cement.h"
#include <driver/ledc.h> //used for pwm
#include <EEPROM.h>
#include <time.h>
#include <sys/time.h>
#include "esp_sntp.h"
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
/** MKS TINYBEE expansion plug pins
 *                ------                                 ------
 *  (BEEPER) 149 | 1  2 | 13 (BTN_ENC)    (SPI MISO) 19 | 1  2 | 18 (SPI SCK)
 *  (LCD_EN)  21 | 3  4 |  4 (LCD_RS)      (BTN_EN1) 14 | 3  4 |  5 (SPI CS)
 *  (LCD_D4)   0   5  6*| 16 (LCD_D5)      (BTN_EN2) 12   5  6 | 23 (SPI MOSI)
 *  (LCD_D6)  15 | 7  8*| 17 (LCD_D7)      (SPI_DET) 34 | 7  8 | RESET
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

/*------------Cat6 Cable 2---
pin----------------wire--------------------------
I2S_DATA -> EXP1_4     Orange (striped orange grounded)
I2S_WS   -> EXP1_6     Green  (striped green grounded)
I2S_BCK  -> EXP1_8     Brown  (striped brown grounded)
24v                    Blue   (striped blue grounded)
*/
/*-------------Board Modifications----------------------
Serial2 rx (gpio 16) & tx (gpio 17) and gpio 4 must be left unused as they are used to route the stepper stream pins to the EXP1 header for easy connection 
  Master Tinybee:
    - The last (far left) HTC595 (serial in parallel out chip) pin Q7' (serial data out) is shorted to gpio 4
    - I2S_WS (gpio 26) pin is shorted to gpio 16 
    - I2S_BCK (gpio 25) pin is shorted to gpio 17
    - On the Serial2 header pin, RX (connected to I2S_WS) is pulled up to ~3.3v with a 10k resistor to ground and a 4.7k resistor to 5v. This helps keep the slave pins from behaving randomly during startup  

  Slave Tinybee:
    - the esp32 has been removed with a heat gun;
    - The line buffers HTC125 2 & 3 have been removed with a heatgun
    - The pins on the removed HTC125s corresponding to EXP1_4,6,8 pins have bin jumpered to to rout the signal back to the gpio pins 4,16,17
    - I2S_DATA (gpio 27) pin is shorted to gpio 4
    - I2S_WS (gpio 26) pin is shorted to gpio 16 
    - I2S_BCK (gpio 25) pin is shorted to gpio 17 
    - The two unsed pins above the "Makerbase" silk screen logo are connected to an improvised 4pin header to control the 2x BTS7960 motor drivers. Pin order is: [PIN_BONU_B_SLAVE | PIN_BONUS_A_SLAVE | 5v | gnd]
*/

#define PIN_BLENDER     PIN_BONUS_A_SLAVE 
#define PIN_SHAKER      PIN_BONUS_B_SLAVE    

#define PIN_PUMP1       FAN_PIN_SLAVE //solid blue        black terminal
#define PIN_PUMP2       FAN1_PIN_SLAVE //striped blue      green terminal

#define PIN_AUX1        HEATER_BED_PIN_SLAVE

#define PIN_RINSE       HEATER_0_PIN_SLAVE
#define PIN_WATER       HEATER_1_PIN_SLAVE


struct Settings{//26 bytes
  byte initialized;//1 byte = flag, used to determine if the eeprom has been initialized with our settings yet
  byte auto_on;//1 byte = flag
  int time_start;//4 bytes = int
  int time_stop;//4 bytes = int
};
static Settings S;

struct Additive{
  char name[10] = "";
  volatile bool timed = false; //dc motors and solenoids use a timer. Otherwise the dose uses a stepper motor
  volatile bool isInline = false;
  float concentration = 1.0f;// grams of additive per liter of extrusion. Extrusion cross sectional area assumed to be 1000mm2
  float mass_flow = 1.0f; //how many g/sec is dosed when turned on. (or steps per gram for stepper dosed additives)
  float default_dose = 1.0f;
  float max_flow = 7.0f;//fastest achievable flowrate g/s
  volatile unsigned long position = 0; //total duration in ms that additive has been dosed
  volatile unsigned long targetPosition = 0;
  volatile bool on = false;
  uint8_t control = -1; //For a stepper controlled additive this is the axis enum ie X_AXIS, for a timed control this is the GPIO pin number
  int ISR_TIMER_ADDITIVE;
  unsigned volatile long end_millis = 276447231;//max long = 276447231. Addatives are dosed using timers and a single stop interrupt. end_millis is used to determine which additive to stop when the interrupt is called
  uint32_t pwm = 0;
};

static const uint8_t NUM_ADDITIVES = 10;
static Additive additives[NUM_ADDITIVES];

#define SEC_PER_DAY 86400

#define blenderPWMChannel 0
#define shakerPWMChannel 1


static const int DOSE_INTERVAL_MS = 500; //time interval to check if dosing should start or stop 
static volatile bool updateWebUi = false;
static volatile bool on = false;
static volatile int print_stage = 0;
static volatile bool print_stage_changed = false;
static volatile bool ramping_blender = false;
static volatile bool ramping_shaker = false;
static volatile int ramp_blender = 0;
static volatile int ramp_shaker = 0;
static volatile int busy = 0;
//static volatile float_t e = 0.0f;

//char arrays to format and send json to the web interface
static char status[1000];//make sure marlin MAX_CMD_SIZE and S2S_RXBUFFERSIZE (in serial2socket.h) are sized correctly
//static char serial2_msg[1000];//serial2 used to allow two tinybees to communicate

//Timer library that creates up to 16 timers from a single hardware timer
ESP32Timer hTimer3(3); //The timer library will use hardware timer 3 which is also used for tone() or beepers by the tinybee
ESP32_ISRTimer ISR_Timer;

static int ISR_TIMER_AUTO_ON;
static int ISR_TIMER_AUTO_OFF;
static int ISR_TIMER_DOSER;

static volatile bool extrude_on = false;
static volatile bool blender_on = false;
static volatile bool shaker_on = false;
static volatile bool aux1_on = false;
static volatile int duty_blender = 15;
static volatile int duty_target_blender = duty_blender;
static volatile int duty_shaker = 23;
static volatile int duty_target_shaker = duty_shaker;
static volatile int duty_aux1 = 50;
static uint32_t pwm_freq = 1000;   
static float printVolume_ml, bufferVolume_ml, wasteVolume_ml;

//static abce_long_t doseTargets;
//static abce_pos_t pos_0;
JsonDocument doc, cmd;

enum PrintStage {
  STAGE_INIT,
  STAGE_MIX,
  STAGE_REVERSE,
  STAGE_PRIME,
  STAGE_READY,
  STAGE_PRINT_ENDED,
  STAGE_CLEANUP
};

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
void IRAM_ATTR alarm_additive_off(){
  unsigned long min = 276447231;
  uint8_t min_index = 0;
  Additive *a = additives;
  for(int i = 0; i < NUM_ADDITIVES; i++,a++){
    if(!a->timed || a->control < 0)continue;
    if(a->on && a->end_millis < min){
      min = a->end_millis;
      min_index = i;
    }
  }
  a = &additives[min_index];
  a->on = false;
  a->end_millis = min;
  a->ISR_TIMER_ADDITIVE = -1;
  WRITE_D_A(a->control,LOW,a->pwm);
  busy--;
}

//Periodically check the extrusion position and dose chemicals if required
void IRAM_ATTR alarm_dose(){// floating point calcs not allowed in ISR (hence doubles)
  if(ramp_blender > 0)ramping_blender = true;
  if(ramp_shaker > 0) ramping_shaker = true;

  if(!extrude_on) return;
  Additive * a = additives;
  for(int i = 0; i < NUM_ADDITIVES; i++, a++){
    if(!a->timed || a->control < 0)continue;
    if(a->on){
      a->position += DOSE_INTERVAL_MS;
    }
    a->on = a->position < a->targetPosition;
    WRITE_D_A(a->control,a->on,a->pwm);
  }
}

/*void IRAM_ATTR alarm_incrementPrintState(){
  print_stage++;
  print_stage_changed = true;
}*/
void IRAM_ATTR alarm_processPrintStage(void *stage){
  print_stage = (int)stage;
  print_stage_changed = true;//flag to process the current print stage in next loop
}
float getInverseMoveDuration(){//get the estimated movement duration for the masters move
  xyze_float_t diff = destination - current_position;
  if (NEAR_ZERO(diff.e)) return -1;//no extrusion
  // Get the linear distance in XYZ
  float cartesian_mm = xyz_float_t(diff).magnitude();
  // If the move is very short, check the E move distance
  if (UNEAR_ZERO(cartesian_mm)) cartesian_mm = ABS(diff.e);
  const float scaled_fr_mm_s = MMS_SCALED(feedrate_mm_s);
  return scaled_fr_mm_s/cartesian_mm;
}
/*Called from gcode when an extrude command is seen
if extrude_on = true the additives and additional extruders are updated to provide appropriate dosage for the extrusion position
*/
void CEMENT::e_destination_changed(bool doseInline){
  if(!extrude_on)return;//
  extrude(doseInline);
}
void CEMENT::extrude(bool doseInline){
  float mix_volume_ml = destination.e +bufferVolume_ml+wasteVolume_ml;//a predifined amount of mix is prepared before extrusion begins. each mm of extrusion assumes 1 mL
  if(mix_volume_ml >= printVolume_ml + wasteVolume_ml){
    mix_volume_ml = printVolume_ml + wasteVolume_ml; //dont create more mix than is necessary
    setShaker(false,duty_shaker); //turn off the vibrator when feed screw is finished
  }
  //mix_volume_ml = min(mix_volume_ml, printVolume_ml + wasteVolume_ml);//dont create more mix than is necessary
  //given the current extruded volume determine how long (in ms) each additive should have been activated for
  Additive *a = additives;
  for(int i = 0; i < NUM_ADDITIVES; i++,a++){
    uint8_t axis = a->control;
    if(axis < 0 || a->concentration < 0.01f || a->mass_flow < 0.01f)continue;//avoid invalid additives
    //if(isSlave != additives[i].slave)continue;//additive is handled by other device
    if(a->timed){
      if(!a->isInline){//ingredients added straight into the mixing tank.
        a->targetPosition = (unsigned long)(mix_volume_ml * a->concentration / a->mass_flow); //mL * mg/mL * ms/mg = ms
      }else if(doseInline){//additives injected into the extrusion pipe
        //chemicals added in this way are added after mixing tank and upstream of the outlet nozzle. They are buffered into the pipe and nozzle before the print starts and can be stopped shortly before the print ends so the final print extrusions purges the chemicals from the nozzle
        a->targetPosition = (unsigned long)(min(destination.e+wasteVolume_ml,printVolume_ml+100.0f) * a->concentration / a->mass_flow);//extra 100mL to ensure the end of the print has the chemical additives
      }
    }else if(axis < NUM_AXES){
      if(!a->isInline){
        destination[axis] = mix_volume_ml * a->concentration/1000.0f; //set destination of I_AXIS (dry cement screw feed) to desired grams (grams = mL Extruded * g/mL)
      }else if(doseInline){
        destination[axis] = min(destination.e+wasteVolume_ml,printVolume_ml+100.0f) * a->concentration/1000.0f; //set destination of I_AXIS (dry cement screw feed) to desired grams (grams = mL Extruded * g/mL)
      }
    }
  }
  //planner.buffer_line_simpler(destination,inv_t);
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

  value = cmd[F("dest")];
  if (!value.isNull()){
    SERIAL_ECHOLNPGM_P(
    LIST_N(DOUBLE(NUM_AXES),
      SP_X_LBL, destination.x,
      SP_Y_LBL, destination.y,
      SP_Z_LBL, destination.y,
      SP_I_LBL, destination.z,
      SP_J_LBL, destination.a,
      SP_K_LBL, destination.b,
      SP_U_LBL, destination.u,
      SP_V_LBL, destination.v,
      SP_W_LBL, destination.w
    )
  );
    return;
  }
  
    value = cmd[F("blend")];
    if (!value.isNull()){
      setBlender(value[0] > 0, value[1]);
      send["blend"][0] = blender_on;
      send["blend"][1] = duty_target_blender;
      reply();
      return;
    }

    value = cmd[F("shaker")];
    if (!value.isNull()){
      setShaker(value[0] > 0, value[1]);
      send["shaker"][0] = shaker_on;
      send["shaker"][1] = duty_target_shaker;
      reply();
      return;
    }
    value = cmd[F("aux1")];
    if (!value.isNull()){
      setAux1(value[0] > 0, value[1]);
      send["aux1"][0] = aux1_on;
      send["aux1"][1] = duty_aux1;
      reply();
      return;
    }

    value = cmd[F("freq")]; 
    if (!value.isNull()){
      pwm_freq = value;
      hal.set_pwm_frequency(PIN_BLENDER,pwm_freq);
      writeBlender();//must call this to reset duty for new frequency
      hal.set_pwm_frequency(PIN_SHAKER,pwm_freq);
      writeShaker();

      send[F("freq")] = pwm_freq;
      reply();
      return;
    }
    

  value = cmd[F("extrude")];
  if(!value.isNull()){
    extrude_on = value;
    send[F("extrude")] = extrude_on;
    zero_axis_I_J_K_U_V_W_E();
    reply(); return;
  }

  value = cmd[F("dose")];
  if (!value.isNull()){
    j = value[0].as<unsigned int>();
    additives[j].concentration = value[1].as<float>();
    additives[j].mass_flow = value[2].as<float>();
    additives[j].default_dose = value[3].as<float>();
    if(!dose_additive(j)) return;//additive was not dosed
    sprintf(status, "Dosed %.1f g of additive %i", additives[j].default_dose, j);
    send["status"] = status;
    reply();
    return;
  }

  value = cmd[F("setPin")];
  if (!value.isNull()){
    uint8_t p = (uint8_t)value[1].as<int>();
    OUT_WRITE(p,value[0].as<int>());
    sprintf(status, "Pin: %i, I2S:%i, value: %i", p,I2S_EXPANDER_PIN_INDEX(p), READ(p));
    SERIAL_ECHOLN(status);
    return;
  }
//P0 {"prep":{"total_mL":704.32,"buffer_mL":300,"waste_mL":700,"mix":[1000,650,100,90,80,70]}}
  value = cmd[F("prep")];/*both*/
  if (!value.isNull()){
    JsonVariant f;
    
    f = value[F("total_mL")];
    if(f.isNull()){throwPrintError();return;}
    printVolume_ml = f.as<float>(); //each mm of extrusion @ cross sectional area of 1000mm2 has 1 mL
    
    f = value[F("buffer_mL")];
    if(f.isNull()){throwPrintError();return;}
    bufferVolume_ml = f.as<float>();
    
    f = value[F("waste_mL")];
    if(f.isNull()){throwPrintError();return;}
    wasteVolume_ml = f.as<float>();

    f = value[F("mix")];
    if(f.isNull()){throwPrintError();return;}
    for (j = 0; j < NUM_ADDITIVES; j++){
      if(j < f.size()){
        additives[j].concentration = f[j];
      }else{
        additives[j].concentration = 0;//unspecified additives set to 0;
      }
    }

    //sprintf(status,"Preparing to print %.1f L of cement",value.as<float>());
    //SERIAL_ECHOLN(status);
    //SERIAL_ECHO_MSG("Preparing");
    //print_stage = 0;
    processPrintStage(STAGE_INIT);
    //printStageChanged();
    return;
  }

  value = cmd[F("mix")];/*both*/ 
  if (!value.isNull()){
    JsonVariant f;
    for (j = 0; j < NUM_ADDITIVES; j++){
      f = value[j][0];
      if(!f.isNull()){
        additives[j].concentration = f.as<float>();
        if(extrude_on){//reset extrude position to 0 to prevent unexptected dosing
          zero_axis_I_J_K_U_V_W_E();
        }
      }
      f = value[j][1];
      if(!f.isNull()){
        additives[j].mass_flow = f.as<float>();
      }
      f = value[j][2];
      if(!f.isNull()){additives[j].default_dose = f.as<float>();}
    }
    printMix();
    reply(); return;
  }
  SERIAL_ECHOLN("Cement cmd not recognised");
}
unsigned long doseTime(float targetVolume, bool isInline){
  unsigned long t = 0;
  unsigned long tMax = 0;
  float flow;
  Additive * a = additives;
  for(int i = 0; i < NUM_ADDITIVES; i++, a++){
      if(a->isInline == isInline && a->concentration > 0.0f && a->mass_flow > 0.01f){//ensure no division by 0 or very long dosing times
        flow = a->timed? a->mass_flow : a->max_flow;//mass flow 
        tMax = max(tMax,(unsigned long)(targetVolume * a->concentration / flow)); //mL * mg/mL * ms/mg = ms
      }
  }
  return tMax;
}

void CEMENT::processPrintStage(int stage){
  print_stage = stage<0?print_stage:stage;//stage defaults to -1 if not passed as an argument
  switch (print_stage){
  case STAGE_INIT:{ /*-------------------------------------------PRINT STARTING-----------------------------------------*/
    setBlender(true, 25);
    setShaker(true, 28);
    setAux1(true);
    // zero the additives
    GcodeSuite::process_subcommands_now(F("M25")); // pause print to to prepare mixture
    unsigned long prepTime_ms = 0;
    float prepVolume_ml = min(bufferVolume_ml, printVolume_ml) + wasteVolume_ml;
    float totalVolume_ml = printVolume_ml + wasteVolume_ml;
    prepTime_ms = doseTime(prepVolume_ml, false); // get dose time for additives that are not inline
    extrude_on = true;
    // unsigned long prepTime = 5000;

    // update ui with mix settings
    PORT_REDIRECT(SerialMask::All); // Serial
    printMix();
    reply();
    // update ui with print volume, prep time and additive masses
    sprintf(status, "echo:Starting %.1fL print. Preparing mixture. Starting in %lusec", printVolume_ml / 1000.0f, prepTime_ms / 1000);
    SERIAL_ECHOLN(status);
    Additive *a = additives;
    for (int i = 0; i < NUM_ADDITIVES; i++, a++){
      if (a->control < 0 || a->concentration < 0.01f || a->mass_flow < 0.01f)
        continue; // avoid invalid additives
      if (a->isInline){
        sprintf(status, "%s:%.1fg priming, %.1fg total", a->name, wasteVolume_ml / 1000.0f * a->concentration, (printVolume_ml + 100.0f) / 1000.0f * a->concentration);
        SERIAL_ECHOLN(status);
      }else{
        sprintf(status, "%s:%.1fg total", a->name, totalVolume_ml / 1000.0f * a->concentration);
        SERIAL_ECHOLN(status);
      }
    }
    // restart print when prep is finished
    feedrate_mm_s = min(30.0f, prepVolume_ml / prepTime_ms * 1000.0f);               // limit feedrate
    prepTime_ms = prepVolume_ml / feedrate_mm_s * 1000.0f;                           // recalculate preptime
    ISR_Timer.setTimeout(prepTime_ms + DOSE_INTERVAL_MS, alarm_processPrintStage,(void*)STAGE_MIX); // add extra dose interval as dosing is only checked and started periodically
    float gCement = prepVolume_ml * additives[0].concentration / 1000.0f;
    current_position.e = 0;
    current_position[I_AXIS] = 0;
    LOOP_DISTINCT_AXES(i){
      destination[i] = current_position[i]; // ensure we only move the axis we want.
    }
    e_destination_changed(false);                       // calculate destination positions for non inline additives. At e=0 (0 + buffer volume + waste volume)
    destination.e = current_position.e - prepVolume_ml; // reverse pump during prep for better mixing
    prepare_line_to_destination();                      // tell steppers to start moving to their destinations
  }break;
  case STAGE_MIX:{ /*----------------------------------------MIXING / RECYCLING----------------------------------------*/
    //mix for certain amount of time while pumping mix back into the top
    SERIAL_ECHOLN("Mixing...");
    unsigned long mixTime = 30000;//ms of mixing time
    feedrate_mm_s = 30;
    destination.e = current_position.e+mixTime/1000*feedrate_mm_s;
    prepare_line_to_destination();
    ISR_Timer.setTimeout(mixTime+1000,alarm_processPrintStage,(void*)STAGE_REVERSE);//extra 1 second to ensure current move is finished
  }break;
  case STAGE_REVERSE:{/*-------------------------------------REVERSE / AIR POCKET REMOVAL ---------------------------------*/
    //reverse pump to remove trapped air and help with priming
    SERIAL_ECHOLN("Purging Air...");
    float mL_reverse = 400.0;//keep some in pump and tube to aid with priming
    destination.e = current_position.e-mL_reverse;
    prepare_line_to_destination();
    ISR_Timer.setTimeout((mL_reverse/feedrate_mm_s+1.0)*1000,alarm_processPrintStage,(void*)STAGE_PRIME);//extra 1 second to ensure current move is finished
  }break;
  case STAGE_PRIME:{/*--------------------------------PRIME NOZZLE AND DOSE INLINE ADDITIVES-----------------------------*/
    unsigned long prepTime_ms = doseTime(wasteVolume_ml,true);
    if(prepTime_ms == 0UL){//if no inline additives skip to next print state
      processPrintStage(STAGE_READY);
      return;
    }
    SERIAL_ECHOLN("Priming...");
    prepTime_ms = max(prepTime_ms,8000UL);//get dose time of inline additives to prime the tube
    //setBlender(true,15);//slow down blender to reduce air intake
    
    feedrate_mm_s = min(30.0f,wasteVolume_ml/prepTime_ms*1000.0f);//limit feedrate
    prepTime_ms = wasteVolume_ml/feedrate_mm_s*1000.0f;//recalculate preptime
    sprintf(status, "Priming: fr:%.1fmm/s, t- %lusec", feedrate_mm_s,prepTime_ms/1000);
    SERIAL_ECHOLN(status);
    //continue print when additives have been dosed
    //LOOP_DISTINCT_AXES(i) {
      //destination[i] = current_position[i];//ensure we only move the axis we want.
    //}
    current_position.e = 0;
    destination.e = 0;
    planner.set_position_mm(current_position);
    //planner.set_e_position_mm(0.0f);
    e_destination_changed(true);//calculate the destination of inline additives for priming the nozzle
    destination.e = current_position.e+wasteVolume_ml;//should be a volume adequate to prime the tube and nozzle
    prepare_line_to_destination();//tell steppers to start moving to their destinations
    ISR_Timer.setTimeout(prepTime_ms+DOSE_INTERVAL_MS, alarm_processPrintStage,(void*)STAGE_READY);//add extra dose interval as dosing is only checked and started periodically
  }break;
  case STAGE_READY:{/*-----------------------------------------READY TO START--------------------------------------------*/
    //Nozzle can still be manually primed at this point. Wait for M24 command to proceed
    current_position.e = 0;//previous priming move must be zeroed. start with a primed nozzle at e position of 0;
    destination.e = 0;
    planner.set_e_position_mm(0.0f);
    SERIAL_ECHOLN("Ready to start print with M24...");//waiting for user (to prime nozzle if necessary) then continue via M24 command
  }break;
  case STAGE_PRINT_ENDED:{/*------------------------------------------PRINT COMPLETE------------------------------------------*///called from M1001.cpp
    extrude_on = false;
    for(int i = 0; i < NUM_ADDITIVES; i++){
      additives[i].position = 0;
      additives[i].targetPosition = 0;
      additives[i].on = false;
      if(additives[i].timed && additives[i].control >=0){
        WRITE_D_A(additives[i].control,LOW,additives[i].pwm);
      }
    }
    zero_axis_I_J_K_U_V_W_E();
    setBlender(false,duty_blender);
    setShaker(false,duty_shaker);
    setAux1(false);
    //processPrintStage(STAGE_CLEANUP);
    return;
  }break;
  case STAGE_CLEANUP:{//cleanup
  }break;
  default:
    break;
  }
}
void CEMENT::reply(){
  serializeJson(doc,status);
  SERIAL_ECHOLN(status);
}
void CEMENT::endPrint(){
  processPrintStage(STAGE_PRINT_ENDED);
}
void CEMENT::throwPrintError(){
  PORT_REDIRECT(SerialMask::All);//Serial
  SERIAL_ECHOLN("Gcode missing prep instructions. Aborting Print");
  GcodeSuite::process_subcommands_now(F("M524"));//Abort sd print
}
double round2(float value) {
   return (int)(value * 100 + 0.5) / 100.0;
}
void CEMENT::printMix(){
  JsonArray mix = doc[F("myPanel")][F("values")][F("mix")].to<JsonArray>();;//.createNestedArray(F("mix"));
  for(int i = 0; i < NUM_ADDITIVES; i++){
    mix[i][0] = round2(additives[i].concentration);
    mix[i][1] = round2(additives[i].mass_flow);
    mix[i][2] = round2(additives[i].default_dose);
  }
}
void CEMENT::initializeAddatives(){
  Additive *a;
  a = additives;
  strcpy(a->name,"Cement");
  a->concentration = 1000.0f;
  a->mass_flow = 20;//desired g/s dosing rate
  a->max_flow = 100;
  a->default_dose = 100.0f;
  a->control = I_AXIS;
  
  a++;//next item (1)
  strcpy(a->name,"Water");
  a->timed = true;
  a->concentration = 1000.0f;
  a->mass_flow = 51.3;// ~19.6 for the spinner, 33.2 for the cone sprayer. This value needs to be calibrated to match the actual mass flow of the valves and tubing in the setup
  a->default_dose = 10.0f;
  a->control = PIN_WATER;
  a->pwm = 128; //solenoid is 12v. send them 50% pwm signal at 24v

  a++;//next item (2)
  strcpy(a->name,"Dose A");
  a->isInline = true; //accelerant added directly into extrusion line
  a->concentration = 1.0f;
  a->default_dose = 5.0f;
  a->control = J_AXIS;
  a->mass_flow = 3.0f;
  
  a++;//next item (3)
  strcpy(a->name,"Dose B");
  a->concentration = 1.0f;
  a->default_dose = 5.0f;
  a->control = K_AXIS;
  a->mass_flow = 3.0f;

  a++;//next item (4)
  strcpy(a->name,"Dose C");
  a->concentration = 0.0f;
  a->default_dose = 5.0f;
  a->control = U_AXIS;
  a->mass_flow = 3.0f;

  a++;//next item (5)
  strcpy(a->name,"Dose D");
  a->concentration = 0.0f;
  a->default_dose = 5.0f;
  a->control = V_AXIS;
  a->mass_flow = 3.0f;

  a++;//next item (6)
  strcpy(a->name,"Dose E");
  a->concentration = 0.0f;
  a->default_dose = 5.0f;
  a->control = W_AXIS;
  a->mass_flow = 3.0f;
  
  a++;//next item (7)
  strcpy(a->name,"Rinse");
  a->timed = true;
  a->concentration = 0.0f;
  a->mass_flow = 51.3;//must be calibrated
  a->default_dose = 10.0f;
  a->control = PIN_RINSE;

  a++;//next item (8)
  strcpy(a->name,"P1");
  a->timed = true;
  a->concentration = 0.0f;
  a->mass_flow = 1.5;//must be calibrated
  a->default_dose = 1.0f;
  a->control = PIN_PUMP1;
  a->pwm = 128; //motors are 12v. send them 50% pwm signal at 24v

  a++;//next item (9)
  strcpy(a->name,"P2");
  a->timed = true;
  a->concentration = 0.0f;
  a->mass_flow = 1.5;//must be calibrated
  a->default_dose = 1.0f;
  a->control = PIN_PUMP2;
  a->pwm = 128; //motors are 12v. send them 50% pwm signal at 24v
}
void CEMENT::buildUi(){
  doc.to<JsonObject>();//clear document
  JsonObject myPanel = myPanel = doc[F("myPanel")].to<JsonObject>();
  myPanel[F("name")] = F("Cement Printer");
  JsonObject ui = myPanel[F("ui")].to<JsonObject>();
  JsonObject item;
  
  /*
  //Date and time
  item = ui[F("time")].to<JsonObject>();
  item[F("type")] = F("datetime-local");
  item[F("label")] = F("Time");
  */

  //Pump command
  item = ui[F("pump")].to<JsonObject>();
  item[F("type")] = F("extruder");

  /*
  //Ability to set an auto start / stop timer
  item = ui[F("auto")].to<JsonObject>();
  item[F("type")] = F("onStartStop");
  item[F("label")] = F("Auto");
  */

  //
  item = ui[F("mix")].to<JsonObject>();
  item[F("type")] = F("table");
  item[F("label")] = F("Mix");
  for(int i = 0; i < NUM_ADDITIVES; i++){
    item[F("rows")][i] = additives[i].name;//
  }
  item[F("cols")] = serialized(F("[\"Conc (g/L)\",\"Flow (g/s)\",\"Dose (g)\"]"));//table column headers
  
  //
  item = ui[F("blend")].to<JsonObject>();
  item[F("type")] = F("onDuty");
  item[F("label")] = F("Blender");

  //
  item = ui[F("shaker")].to<JsonObject>();
  item[F("type")] = F("onDuty");
  item[F("label")] = F("Shaker");

  //
  item = ui[F("freq")].to<JsonObject>();
  item[F("type")] = F("number");
  item[F("label")] = F("PWM Frequency");
  item[F("append")] = F("hz");

  item = ui[F("aux1")].to<JsonObject>();
  item[F("type")] = F("onDuty");
  item[F("label")] = F("Aux1");

  item = ui[F("setPin")].to<JsonObject>();
  item[F("type")] = F("onDuty");
  item[F("label")] = F("Set Pin");
}
void CEMENT::buildValues(){
  doc.to<JsonObject>();//clear document
  
  JsonObject myPanel = doc[F("myPanel")].to<JsonObject>();
  JsonObject values = myPanel[F("values")].to<JsonObject>();//myPanel.createNestedObject(F("values"));
  //values[F("time")] = getTime();
  //values[F("auto")] = serialized(F("[0,1,1]"));
  values[F("extrude")] = extrude_on;
  values[F("blend")][0] = blender_on;
  values[F("blend")][1] = duty_blender;
  values[F("shaker")][0] = shaker_on;
  values[F("shaker")][1] = duty_shaker;
  values[F("aux1")][0] = aux1_on;
  values[F("aux1")][1] = duty_aux1;
  values[F("freq")] = pwm_freq;
  
  values[F("setPin")][0] = 1;
  values[F("setPin")][1] = 150;
  printMix();
}
void CEMENT::setup(){
    //Set Pin states
    initializeAddatives();
    //setupClock();
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
    //ISR_TIMER_AUTO_ON = ISR_Timer.setInterval(SEC_PER_DAY*1000,alarm_autoOn);                      //auto on timer
    //ISR_TIMER_AUTO_OFF = ISR_Timer.setInterval(SEC_PER_DAY*1000,alarm_autoOff);                    //auto off timer
    ISR_TIMER_DOSER = ISR_Timer.setInterval(DOSE_INTERVAL_MS, alarm_dose);
    //setAutoTimers();//Auto on & off timers were initialized with arbitrary values above. Update them based on current time
    /*
    ledcSetup(blenderPWMChannel, pwm_freq, 7);//3khz 7bits (0-128)
    ledcAttachPin(PIN_BLENDER, blenderPWMChannel);

    ledcSetup(auxPWMChannel, pwm_freq, 7);//3khz 7bits (0-128)
    ledcAttachPin(PIN_AUX, auxPWMChannel);

    writeOutputs();
    */
   
   wasteVolume_ml =0; printVolume_ml = 1000 /*init to 0?*/; bufferVolume_ml = 0;
   hal.set_pwm_frequency(PIN_BLENDER,pwm_freq);
   hal.set_pwm_frequency(PIN_SHAKER,pwm_freq);
   hal.set_pwm_frequency(PIN_AUX1,pwm_freq);
   hal.set_pwm_frequency(PIN_PUMP1,pwm_freq);
   hal.set_pwm_frequency(PIN_PUMP2,pwm_freq);
   hal.set_pwm_frequency(PIN_WATER,20000);
   CEMENT::writeOutputs();

  /*position zeroing*/
  LOOP_DISTINCT_AXES(i) {
    destination[i] = current_position[i];//ensure destination is the same as current position at boot.
  }
  //CEMENT::set_additive_axis_steps;
}

void CEMENT::zero_axis_I_J_K_U_V_W_E(){
  LOOP_DISTINCT_AXES(i){
    if(i > Z_AXIS){
      current_position[i] = 0;
      destination[i] = 0;
    }
  }
  planner.refresh_positioning();//allow planner to recalculate settings
  //planner.set_machine_position_mm(pos_0);//set the stepper motor positions.
}
bool CEMENT::isBusy(){
  return busy>0;
}
void CEMENT::loop(){
  if(updateWebUi){
    updateWebUi = false;
    CEMENT::handleCommand(F("{\"fetch_values\":1}"));//auto mode has turned the equipment on or off. Update the UI
    //CEMENT::handleCommand(114,"");
  }
  
  if(print_stage_changed){
    print_stage_changed = false;
    processPrintStage();
  }
  //motor pwm changes are eased in to prevent jerking
  if(ramping_blender){
    setBlender(blender_on, duty_target_blender);
  }
  if(ramping_shaker){
    setShaker(shaker_on, duty_target_shaker);
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
    CEMENT::handleCommand(F("{\"time\":\"\"}"));//output time to serial 
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
  writeBlender();
  writeShaker();
}
void CEMENT::setBlender(bool on, int duty){
  ramping_blender = false;
  duty_target_blender = duty;
  int steps = 8;
  if(ramp_blender == 0){
    if(!on && !blender_on){//if changing duty while motor is off do it imediately
      duty_blender = duty;
      return;
    }else{//start transition from on to off or off to on. Update current duty to reflect on state
      duty_blender *= blender_on;
    }
  }
  blender_on = on;
  if(ramp_blender >= steps){//end of transition
      duty_blender = duty_target_blender;
      ramp_blender = 0;
      writeBlender();
  }else{
    ramp_blender++;
    int d = (ramp_blender*duty_target_blender*on + (steps-ramp_blender)*duty_blender)/steps;
    hal.set_pwm_duty(PIN_BLENDER,(d*255+99)/100);
    //ledcWrite(blenderPWMChannel,d*128/100);
  }
}
void CEMENT::setShaker(bool on, int duty){
  ramping_shaker = false;
  duty_target_shaker = duty;
  int steps = 8;
  if(ramp_shaker == 0){
    if(!on && !shaker_on){//if changing duty while motor is off do it imediately
      duty_shaker = duty;
      return;
    }else{//start transition from on to off or off to on. Update current duty to reflect on state
      duty_shaker *= shaker_on;
    }
  }
  shaker_on = on;
  if(ramp_shaker >= steps){//end of transition
      duty_shaker = duty_target_shaker;
      ramp_shaker = 0;
      writeShaker();
  }else{
    ramp_shaker++;
    int d = (ramp_shaker*duty_target_shaker*on + (steps-ramp_shaker)*duty_shaker)/steps;
    hal.set_pwm_duty(PIN_SHAKER,(d*255+99)/100);
    //ledcWrite(shakerPWMChannel,d*128/100);
  }
}
void CEMENT::setAux1(bool on, int duty){
  aux1_on = on;
  duty_aux1 = duty;
  hal.set_pwm_duty(PIN_AUX1,(duty_aux1*255+99)/100*aux1_on);
}
/*Start a timed output to dose a desired amount of chemical*/
bool CEMENT::dose_additive(uint8_t index){
  if(!(index < NUM_ADDITIVES && index >= 0)) return false; //return on invalid input 
  
  Additive *a = &additives[index];

  if(a->default_dose<=0 || a->mass_flow <= 0 || a->control < 0) return false;

  if (a->timed){
    int doseDuration = a->default_dose/a->mass_flow*1000;
    if(a->default_dose == 69.0f){//69 is the secret code to calibrate the pump
      doseDuration = 20.0f;//20 second timer
    }
    if(doseDuration > 60000) return false;//prevent doses > 30s
    a->on = true;
    a->end_millis = millis()+doseDuration; //end_millis is used in the interrupt alarm_additive_off to determine which additive to stop when multiple are active
    WRITE_D_A(a->control,HIGH,a->pwm);
    busy++;
    a->ISR_TIMER_ADDITIVE = ISR_Timer.setTimeout(doseDuration, alarm_additive_off);
  }else{
    destination = current_position;//insure axis destinations match current position
    planner.set_position_mm(current_position);//make sure planner is using current position
    uint8_t axis = a->control;
    feedrate_mm_s = 5.0;
    if(a->default_dose==70.0){//70 indicates that the value in a->mass_flow is the mass that was dosed in the calibration cycle
      planner.settings.axis_steps_per_mm[axis] = 4748.3871f*60.0f/a->mass_flow;
      zero_axis_I_J_K_U_V_W_E();//save changes
      a->default_dose = 10.0f;
      a->mass_flow = 3.0f;
      printMix(); reply(); 
      return false;
    }else if(a->default_dose==69.0f){//69 is the secret code to calibrate the pump
      float cal_units = 4748.3871f*60.0f/planner.settings.axis_steps_per_mm[axis];//calibration steps desired (motor_steps * micro_steps * gear_ratio * desired rotations (60) / steps per unit)
      destination[axis] = current_position[axis] + cal_units;
      feedrate_mm_s = cal_units / 30.0f;//set feedrate to 2 rps
      a->default_dose = 70.0f; //indicate that next value is the measured dose
      printMix(); reply();
    }else{
      feedrate_mm_s = a->mass_flow;
      destination[axis] = current_position[axis] + a->default_dose;
    }
    prepare_line_to_destination();
  }
  return true;
}

void CEMENT::writeBlender(){
  hal.set_pwm_duty(PIN_BLENDER,(duty_blender*255+99)/100*blender_on);//+99 is added to get the ceiling of the division result 
}

void CEMENT::writeShaker(){
  hal.set_pwm_duty(PIN_SHAKER,(duty_shaker*255+99)/100*shaker_on);
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

