#ifndef WCK_H
#define WCK_H

#include <Arduino.h>

class WCK {
public:
    WCK(HardwareSerial* serial);
    
    // Commands
    int readPos(uint8_t id);
    int pos(uint8_t id, uint8_t torque, uint8_t target);
    void posGroup(uint8_t lastId, uint8_t torque, uint8_t* target);
    int passivate(uint8_t id);
    
private:
    HardwareSerial* _serial;
    void sendCmd(uint8_t data1, uint8_t data2);
    int read(uint8_t timeout_ms = 50);
};

#endif
