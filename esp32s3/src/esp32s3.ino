#include <Arduino.h>
#include "WCK.h"
#include "PK.h"
#include "Calib.h"
#include "Motion.h"

// Define Serial for WCK (ESP32 S3 typically has 3 hardware serials)
// Use UART1 or UART2, define your RX and TX pins connected to WCK servos
#define WCK_RX 5
#define WCK_TX 4

HardwareSerial wckSerial(1); // Using UART1
WCK wck(&wckSerial);
PK walkEngine(&wck);
Calib calib(&wck);
MotionController motion(&wck);

void setup() {
    Serial.begin(115200);
    // Initialize WCK Serial
    wckSerial.begin(115200, SERIAL_8N1, WCK_RX, WCK_TX);
    
    // Load calibration from non-volatile preferences
    calib.loadZero();

    delay(2000);
    Serial.println("ESP32-S3 HUNO Controller");
    Serial.println("Commands:");
    Serial.println("  w / start : start walk");
    Serial.println("  s / stop  : stop walk");
    Serial.println("  c : run calibration menu");
    Serial.println("  params : print walk parameters");
    Serial.println("  f 10 : set frequency to 10 (similarly: h, y, sh, sl, ss, as, d, sc)");
    Serial.println("  sign 2 : toggle sign for joint ID 2");
    Serial.println("  m [file] : play motion file from LittleFS (e.g. m /motion.json)");
    Serial.println("  p : passivate motors (for motion gen)");
    Serial.println("  r : read current pose (for motion gen)");
    Serial.println("  z : goto zero");
    
    motion.begin(); // init LittleFS
    walkEngine.begin();
}

void loop() {
    walkEngine.update(); // non-blocking walk loop
    
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        int spaceIdx = cmd.indexOf(' ');
        String action = cmd;
        String arg = "";
        if (spaceIdx != -1) {
            action = cmd.substring(0, spaceIdx);
            arg = cmd.substring(spaceIdx + 1);
            action.trim();
            arg.trim();
        }

        if (action == "w" || action == "start") {
            walkEngine.startWalk();
            Serial.println("Walking started");
        } else if (action == "s" || action == "stop") {
            walkEngine.stopWalk();
            Serial.println("Walking stopped");
        } else if (action == "params") {
            walkEngine.printParams();
        } else if (action == "f") {
            walkEngine.f = arg.toFloat();
            Serial.print("f = "); Serial.println(walkEngine.f);
        } else if (action == "h") {
            walkEngine.robot_height = arg.toFloat();
            Serial.print("h = "); Serial.println(walkEngine.robot_height);
        } else if (action == "y") {
            walkEngine.shift_y = arg.toFloat();
            Serial.print("y = "); Serial.println(walkEngine.shift_y);
        } else if (action == "sh") {
            walkEngine.step_height = arg.toFloat();
            Serial.print("sh = "); Serial.println(walkEngine.step_height);
        } else if (action == "sl") {
            walkEngine.step_length = arg.toFloat();
            Serial.print("sl = "); Serial.println(walkEngine.step_length);
        } else if (action == "ss") {
            walkEngine.side_step = arg.toFloat();
            Serial.print("ss = "); Serial.println(walkEngine.side_step);
        } else if (action == "as") {
            walkEngine.arm_swing = arg.toFloat();
            Serial.print("as = "); Serial.println(walkEngine.arm_swing);
        } else if (action == "d") {
            walkEngine.direction = arg.toFloat();
            Serial.print("d = "); Serial.println(walkEngine.direction);
        } else if (action == "sc") {
            walkEngine.scale = arg.toFloat();
            Serial.print("sc = "); Serial.println(walkEngine.scale);
        } else if (action == "sign") {
            int id = arg.toInt();
            walkEngine.toggleSign(id);
        } else if (action == "c") {
            walkEngine.stopWalk();
            calib.runCalibrationMenu();
        } else if (action == "p") {
            walkEngine.stopWalk();
            motion.setPassiveMode();
        } else if (action == "r") {
            motion.printCurrentPose();
        } else if (action == "z") {
            walkEngine.stopWalk();
            motion.gotoZero();
        } else if (action == "m") {
            walkEngine.stopWalk();
            Serial.println("Playing motion: " + arg);
            motion.playMotion(arg.c_str());
        }
    }
}
