#include "Calib.h"
#include <Preferences.h>

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
    Preferences prefs;
    prefs.begin("huno", false);
    prefs.putBytes("zero", zero_offsets, NUM_JOINTS);
    prefs.end();
    Serial.println("Zero positions saved to flash.");
}

void Calib::printJoints() {
    for(int i = 0; i < NUM_JOINTS; i++) {
        int pos = _wck->readPos(i);
        Serial.print("Joint ID ");
        Serial.print(i);
        Serial.print(" -> ");
        Serial.println(pos);
    }
}

void Calib::runCalibrationMenu() {
    loadZero(); // Load from flash if available
    _wck->posGroup(15, 4, zero_offsets);
    
    Serial.println("\n=== HUNO Joint Calibration Tool ===");
    Serial.println("Select Joint ID (0-15) to calibrate, 's' to save, 'x' to exit.");
    
    int current_id = -1;
    
    while(true) {
        if(Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            input.trim();
            if(input.length() == 0) continue;
            
            if(input == "x") {
                Serial.println("Exiting calibration menu.");
                break;
            } else if(input == "s") {
                saveZero();
            } else if(input == "w") { // up
                if(current_id >= 0 && current_id < NUM_JOINTS) {
                    zero_offsets[current_id]++;
                    _wck->pos(current_id, 4, zero_offsets[current_id]);
                    Serial.print("ID "); Serial.print(current_id); Serial.print(" -> "); Serial.println(zero_offsets[current_id]);
                }
            } else if(input == "z") { // down
                if(current_id >= 0 && current_id < NUM_JOINTS) {
                    zero_offsets[current_id]--;
                    _wck->pos(current_id, 4, zero_offsets[current_id]);
                    Serial.print("ID "); Serial.print(current_id); Serial.print(" -> "); Serial.println(zero_offsets[current_id]);
                }
            } else {
                int id = input.toInt();
                if((id >= 0 && id < NUM_JOINTS) || input == "0") {
                    current_id = id;
                    Serial.print("Calibrating ID ");
                    Serial.println(current_id);
                    Serial.println("Send 'w' to increase, 'z' to decrease.");
                }
            }
        }
        delay(50);
    }
}
