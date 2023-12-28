#include "Arduino.h"

class CEMENT{
public:
    CEMENT();
    static void setup();
    static void handleCommand(const int8_t c, String val);
    static void loop();
    static void extrude(float_t ex);
    static void IRAM_ATTR writeOutputs();
    static void IRAM_ATTR writeBlender();
    static void IRAM_ATTR writeWater();
    static void IRAM_ATTR writeDoseA();
    static void IRAM_ATTR writeDoseB();
    static void IRAM_ATTR writeDoseC();
    static void IRAM_ATTR writeAux();
    static void IRAM_ATTR waterOn(bool on);
    static void IRAM_ATTR doseAOn(bool on);
    static void IRAM_ATTR doseBOn(bool on);
    static void IRAM_ATTR doseCOn(bool on);
private:
    static void initializeAddatives();
    static void saveSettings();//store settings in eeprom
    static void loadSettings();//load settings from eeprom

    //Auto settings
    static void setupClock();
    static char* getTimeCurrent();
    static void setTimeCurrent(const char *sDateTime);
    static void timeSyncCallback(struct timeval *tv);

    static void setAutoTimers();
    static void setAuto(bool on, int start, int stop);

    static void setBlender(bool on, int duty = 50);
    static void setAux(bool on, int duty = 50);
};
