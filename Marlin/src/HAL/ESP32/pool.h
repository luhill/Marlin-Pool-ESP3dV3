#include "Arduino.h"

class POOL{
public:
    POOL();
    static void setup(uint32_t cycle_period_ms = 2000);
    static void handleCommand(const int8_t c, String val);
    static void loop();
    static void IRAM_ATTR writeChlorine();
    static void IRAM_ATTR writeIon();
    static void IRAM_ATTR writeAcid();
    static void IRAM_ATTR writeFloc();
    static void IRAM_ATTR writeOutputs();
    static void IRAM_ATTR pumpOn(bool on);
    static void IRAM_ATTR boosterOn(bool on);
    static void IRAM_ATTR pump_12vOn(bool on);
    static void IRAM_ATTR alarm_attachOnButton_a();
    static void IRAM_ATTR alarm_onButtonPushed_a();
    static void IRAM_ATTR alarm_attachOnButton_b();
    static void IRAM_ATTR alarm_onButtonPushed_b();
private:
    static void saveSettings();//store settings in eeprom
    static void loadSettings();//load settings from eeprom

    //Auto settings
    static void setupClock();
    static char* getTimeCurrent();
    static void setTimeCurrent(const char *sDateTime);
    static void timeSyncCallback(struct timeval *tv);

    static void setAutoTimers();
    static void setAuto_a(bool on, int start, int stop);
    static void setAuto_b(bool on, int start, int stop);

    //chlorinator settings
    static void setChlorine(bool on, int duty = 50/*0-100%*/, bool write=true);
    static void cycleChlorine(int period_sec = 30000);//30 sec default

    //ionizer settings
    static void setIon(bool on, int duty = 50, bool write=true);
    static void cycleIon(int period_sec = 30000);//30 sec default

    static void setAcid(bool on, int duty = 50);
    static void setFloc(bool on, int duty = 50);
};
