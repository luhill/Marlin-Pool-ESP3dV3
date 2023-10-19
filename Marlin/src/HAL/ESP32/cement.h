#include "Arduino.h"

class CEMENT{
public:
    CEMENT();
    static void setup();
    static void handleCommand(const int8_t c, String val);
    static void loop();
    static void writeOutputs();
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
};
