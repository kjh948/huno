#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <WebSerialLite.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include "WCK.h"
#include "PK.h"
#include "Calib.h"
#include "Motion.h"
#include "wifi_credentials.h"
#include "web_page.h"

// Define Serial for WCK (ESP32 S3 typically has 3 hardware serials)
// Use UART1 or UART2, define your RX and TX pins connected to WCK servos
#define WCK_RX 6
#define WCK_TX 5

HardwareSerial wckSerial(1); // Using UART1
WCK wck(&wckSerial);
PK walkEngine(&wck);
Calib calib(&wck);
MotionController motion(&wck);

AsyncWebServer server(80);

void printHelp(bool isWebSerial = false) {
    String out = "";
    out += "=== HUNO Commands ===\n";
    out += "  [Enter]  : show this help\n";
    out += "  w/start  : start walk\n";
    out += "  s/stop   : stop walk\n";
    out += "  c        : calibration menu\n";
    out += "  z        : goto zero\n";
    out += "  p        : passivate motors\n";
    out += "  r        : read current pose\n";
    out += "  sign <id>: toggle joint sign\n";
    out += "  m <file> : play motion file\n";
    out += "--- Walk Parameters (current / default) ---\n";

    auto paramLine = [&](const char* cmd, const char* name, float cur, float def) {
        out += "  ";
        out += cmd;
        out += String("       ").substring(0, 5 - strlen(cmd));
        out += ": ";
        out += name;
        out += " = ";
        out += String(cur, 2);
        out += " (def: ";
        out += String(def, 2);
        out += ")\n";
    };

    paramLine("f",  "frequency   ", walkEngine.f,            8.0f);
    paramLine("h",  "height      ", walkEngine.robot_height, 0.5f);
    paramLine("y",  "shift_y     ", walkEngine.shift_y,      0.2f);
    paramLine("sh", "step_height ", walkEngine.step_height,  0.4f);
    paramLine("sl", "step_length ", walkEngine.step_length,  0.2f);
    paramLine("ss", "side_step   ", walkEngine.side_step,    0.0f);
    paramLine("as", "arm_swing   ", walkEngine.arm_swing,    1.0f);
    paramLine("d",  "direction   ", walkEngine.direction,   -1.0f);
    paramLine("sc", "scale       ", walkEngine.scale,       30.0f);

    out += "=====================";

    if (isWebSerial) WebSerial.println(out);
    else Serial.println(out);
}

void processCommand(String cmd, bool isWebSerial = false) {
    cmd.trim();
    if (cmd.length() == 0) {
        printHelp(isWebSerial);
        return;
    }
    
    int spaceIdx = cmd.indexOf(' ');
    String action = cmd;
    String arg = "";
    if (spaceIdx != -1) {
        action = cmd.substring(0, spaceIdx);
        arg = cmd.substring(spaceIdx + 1);
        action.trim();
        arg.trim();
    }

    auto printMsg = [&](String msg) {
        if (isWebSerial) WebSerial.println(msg);
        else Serial.println(msg);
    };

    if (action == "w" || action == "start") {
        walkEngine.startWalk();
        printMsg("Walking started");
    } else if (action == "s" || action == "stop") {
        walkEngine.stopWalk();
        printMsg("Walking stopped");
    } else if (action == "params") {
        walkEngine.printParams(); 
    } else if (action == "f") {
        walkEngine.f = arg.toFloat();
        printMsg("f = " + String(walkEngine.f));
    } else if (action == "h") {
        walkEngine.robot_height = arg.toFloat();
        printMsg("h = " + String(walkEngine.robot_height));
    } else if (action == "y") {
        walkEngine.shift_y = arg.toFloat();
        printMsg("y = " + String(walkEngine.shift_y));
    } else if (action == "sh") {
        walkEngine.step_height = arg.toFloat();
        printMsg("sh = " + String(walkEngine.step_height));
    } else if (action == "sl") {
        walkEngine.step_length = arg.toFloat();
        printMsg("sl = " + String(walkEngine.step_length));
    } else if (action == "ss") {
        walkEngine.side_step = arg.toFloat();
        printMsg("ss = " + String(walkEngine.side_step));
    } else if (action == "as") {
        walkEngine.arm_swing = arg.toFloat();
        printMsg("as = " + String(walkEngine.arm_swing));
    } else if (action == "d") {
        walkEngine.direction = arg.toFloat();
        printMsg("d = " + String(walkEngine.direction));
    } else if (action == "sc") {
        walkEngine.scale = arg.toFloat();
        printMsg("sc = " + String(walkEngine.scale));
    } else if (action == "sign") {
        int id = arg.toInt();
        walkEngine.toggleSign(id);
    } else if (action == "c") {
        walkEngine.stopWalk();
        calib.startCalibration(isWebSerial);
    } else if (action == "p") {
        walkEngine.stopWalk();
        motion.setPassiveMode(isWebSerial);
    } else if (action == "r") {
        motion.printCurrentPose(isWebSerial);
    } else if (action == "z") {
        walkEngine.stopWalk();
        motion.gotoZero();
    } else if (action == "m") {
        walkEngine.stopWalk();
        printMsg("Playing motion: " + arg);
        motion.playMotion(arg.c_str());
    }
}

void recvMsg(uint8_t *data, size_t len) {
    String d = "";
    for (int i = 0; i < len; i++) {
        d += (char)data[i];
    }
    d.trim();
    if (calib.isCalibActive()) {
        calib.processCalibInput(d);
    } else {
        processCommand(d, true);
    }
}

// Functions maintained but disabled from routing as requested
void handleRoot(AsyncWebServerRequest *request) {
    request->send(200, "text/html", html_page);
}

void handleGetParams(AsyncWebServerRequest *request) {
    JsonDocument doc;
    doc["f"] = walkEngine.f;
    doc["h"] = walkEngine.robot_height;
    doc["y"] = walkEngine.shift_y;
    doc["sh"] = walkEngine.step_height;
    doc["sl"] = walkEngine.step_length;
    doc["ss"] = walkEngine.side_step;
    doc["as"] = walkEngine.arm_swing;
    doc["d"] = walkEngine.direction;
    doc["sc"] = walkEngine.scale;
    
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

void handleSetParam(AsyncWebServerRequest *request) {
    if (request->hasParam("param") && request->hasParam("value")) {
        String param = request->getParam("param")->value();
        float val = request->getParam("value")->value().toFloat();
        
        if (param == "f") walkEngine.f = val;
        else if (param == "h") walkEngine.robot_height = val;
        else if (param == "y") walkEngine.shift_y = val;
        else if (param == "sh") walkEngine.step_height = val;
        else if (param == "sl") walkEngine.step_length = val;
        else if (param == "ss") walkEngine.side_step = val;
        else if (param == "as") walkEngine.arm_swing = val;
        else if (param == "d") walkEngine.direction = val;
        else if (param == "sc") walkEngine.scale = val;
        
        request->send(200, "text/plain", "OK");
    } else {
        request->send(400, "text/plain", "Bad Request");
    }
}

void handleCmd(AsyncWebServerRequest *request) {
    if (request->hasParam("action")) {
        String action = request->getParam("action")->value();
        if (action == "w" || action == "start") {
            walkEngine.startWalk();
        } else if (action == "s" || action == "stop") {
            walkEngine.stopWalk();
        }
        request->send(200, "text/plain", "OK");
    } else {
        request->send(400, "text/plain", "Bad Request");
    }
}

void setup() {
    Serial.begin(115200);
    // Initialize WCK Serial
    wckSerial.begin(115200, SERIAL_8N1, WCK_RX, WCK_TX);
    
    // Load calibration from non-volatile preferences
    calib.loadZero();

    delay(2000);
    Serial.println("ESP32-S3 HUNO Controller");
    printHelp();
    
    motion.begin(); // init LittleFS
    walkEngine.begin();

    // Wi-Fi and Web Server init
    Serial.println();
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("WiFi connected. IP address: ");
        Serial.println(WiFi.localIP());
        
        if (MDNS.begin("huno")) {
            Serial.println("MDNS responder started (huno.local)");
        }

        // Redirect root to webserial
        server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
            request->redirect("/webserial");
        });

        // Web Interface routes (Disabled as requested, kept for reference)
        /*
        server.on("/", HTTP_GET, handleRoot);
        server.on("/getParams", HTTP_GET, handleGetParams);
        server.on("/set", HTTP_GET, handleSetParam);
        server.on("/cmd", HTTP_GET, handleCmd);
        */

        // WebSerial init
        WebSerial.begin(&server);
        WebSerial.onMessage(recvMsg);

        server.begin();
        Serial.println("HTTP server started");
    } else {
        Serial.println("WiFi connection failed.");
    }
}

void loop() {
    walkEngine.update(); // non-blocking walk loop
    
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (calib.isCalibActive()) {
            calib.processCalibInput(cmd);
        } else {
            processCommand(cmd, false);
        }
    }
}
