#include "Arduino.h"

class CHILIPAD{
public:
    CHILIPAD();
    static void setup(uint32_t cycle_period_ms = 2000);
    static void handleCommand(const int8_t c, String val);
    static void loop();
    static void IRAM_ATTR writePump();
    static void IRAM_ATTR writeFan();
    static void IRAM_ATTR writeLed();
    static void IRAM_ATTR writeOutputs();
    static void IRAM_ATTR alarm_attachOnButton();
    static void IRAM_ATTR alarm_onButtonPushed();
    static void IRAM_ATTR alarm_low_water();
private:
    static void saveSettings();//store settings in eeprom
    static void loadSettings();//load settings from eeprom

    //Auto settings
    static void setupClock();
    static char* getTimeCurrent();
    static void setTimeCurrent(const char *sDateTime);
    static void timeSyncCallback(struct timeval *tv);

    static void setAutoTimers();
    static void setAuto(bool on, int start, int stop);

    //chlorinator settings
    static void setPump(bool on, int duty = 50);

    //ionizer settings
    static void setFan(bool on, int duty = 50);
    
};
