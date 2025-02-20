#include "Arduino.h"
#include "../../core/types.h" 
class CEMENT{
public:
    CEMENT();
    static void setup();
    static void handleCommand(String val);
    static void reply();
    static void loop();
    static void e_destination_changed(bool doseInline = true);
    static void endPrint();
    static void IRAM_ATTR writeOutputs();//functions used in ISR interrupts must be public
    static bool isBusy();
    //static void progressPrintStage();
    
private:
    static void buildUi();
    static void buildValues();
    
    static void saveSettings();//store settings in eeprom
    static void loadSettings();//load settings from eeprom

    //Auto settings
    static void setupClock();
    static char* getTime();
    static void setTime(const char *sDateTime);
    static void timeSyncCallback(struct timeval *tv);

    static void setAutoTimers();
    static void setAuto(bool on, int start, int stop);
};
