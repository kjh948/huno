#include "WCK.h"

WCK::WCK(HardwareSerial* serial) : _serial(serial) {}

void WCK::sendCmd(uint8_t data1, uint8_t data2) {
    uint8_t chksum = (data1 ^ data2) & 127;
    // clear RX buffer
    while(_serial->available()) _serial->read();
    
    _serial->write(255); // Header 0xFF
    _serial->write(data1);
    _serial->write(data2);
    _serial->write(chksum);
}

int WCK::read(uint8_t timeout_ms) {
    unsigned long start = millis();
    while (_serial->available() < 2) {
        if (millis() - start > timeout_ms) return -1;
        yield();
    }
    uint8_t data1 = _serial->read();
    uint8_t data2 = _serial->read();
    return data2; // data1 is load/error, data2 is position/value
}

int WCK::readPos(uint8_t id) {
    sendCmd(0xA0 | id, 0x00);
    return read();
}

int WCK::pos(uint8_t id, uint8_t torque, uint8_t target) {
    sendCmd((torque << 5) | id, target);
    return read();
}

void WCK::posGroup(uint8_t lastId, uint8_t torque, uint8_t* target) {
    while(_serial->available()) _serial->read();
    
    _serial->write(0xFF); // Header
    _serial->write((torque << 5) | 0x1F); // Sync move command
    _serial->write(lastId + 1); // Length
    
    uint8_t chksum = 0;
    for(int i = 0; i <= lastId; i++) {
        _serial->write(target[i]);
        chksum ^= target[i];
    }
    _serial->write(chksum & 0x7F);
}

int WCK::passivate(uint8_t id) {
    // torque=0 releases the motor
    sendCmd((0 << 5) | id, 0x00);
    return read();
}
