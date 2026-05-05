#include "Calib.h"
#include <Preferences.h>
#include <WebSerialLite.h>

Calib::Calib(WCK* wck) : _wck(wck) {
}

void Calib::loadZero() {
    Preferences prefs;
    prefs.begin("huno", true); // read-only
    if(prefs.getBytesLength("zero") == NUM_JOINTS) {
        prefs.getBytes("zero", zero_offsets, NUM_JOINTS);
    }
    prefs.end();
}

void Calib::saveZero() {
    _print("Zero positions saved to flash.");
    Preferences prefs;
    prefs.begin("huno", false);
    prefs.putBytes("zero", zero_offsets, NUM_JOINTS);
    prefs.end();
}

void Calib::printJoints() {
    for(int i = 0; i < NUM_JOINTS; i++) {
        int pos = _wck->readPos(i);
        String msg = "Joint ID " + String(i) + " -> " + String(pos);
        _print(msg);
    }
}

void Calib::_print(String msg) {
    if (_isWebSerial) WebSerial.println(msg);
    else Serial.println(msg);
}

void Calib::startCalibration(bool isWebSerial) {
    loadZero();
    _wck->posGroup(15, 4, zero_offsets);
    _calibMode = true;
    _calibCurrentId = -1;
    _isWebSerial = isWebSerial;
    _print("=== HUNO Joint Calibration Tool ===");
    _print("Select Joint ID (0-15), 'w'=+1, 'z'=-1, 's'=save, 'x'=exit.");
}

void Calib::processCalibInput(String input) {
    input.trim();
    if(input.length() == 0) {
        _print("[Calib] ID: " + String(_calibCurrentId) + " | 'w'=+1, 'z'=-1, 's'=save, 'x'=exit");
        return;
    }

    if(input == "x") {
        _print("Exiting calibration menu.");
        _calibMode = false;
    } else if(input == "s") {
        saveZero();
    } else if(input == "w") {
        if(_calibCurrentId >= 0 && _calibCurrentId < NUM_JOINTS) {
            zero_offsets[_calibCurrentId]++;
            _wck->pos(_calibCurrentId, 4, zero_offsets[_calibCurrentId]);
            _print("ID " + String(_calibCurrentId) + " -> " + String(zero_offsets[_calibCurrentId]));
        } else {
            _print("[Calib] No joint selected. Enter joint ID first.");
        }
    } else if(input == "z") {
        if(_calibCurrentId >= 0 && _calibCurrentId < NUM_JOINTS) {
            zero_offsets[_calibCurrentId]--;
            _wck->pos(_calibCurrentId, 4, zero_offsets[_calibCurrentId]);
            _print("ID " + String(_calibCurrentId) + " -> " + String(zero_offsets[_calibCurrentId]));
        } else {
            _print("[Calib] No joint selected. Enter joint ID first.");
        }
    } else {
        int id = input.toInt();
        if((id >= 0 && id < NUM_JOINTS) || input == "0") {
            _calibCurrentId = id;
            _print("Calibrating ID " + String(_calibCurrentId));
            _print("Send 'w' to increase, 'z' to decrease.");
        } else {
            _print("[Calib] Unknown input: " + input);
        }
    }
}
