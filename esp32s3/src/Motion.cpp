#include "Motion.h"
#include <LittleFS.h>
#include <ArduinoJson.h> // Make sure to install ArduinoJson via Library Manager

MotionController::MotionController(WCK* wck) : _wck(wck) {
    for(int i=0; i<NUM_JOINTS; i++) {
        cur_pose[i] = zero_offsets[i];
        joint_signs[i] = 1.0;
    }
    joint_signs[4] = -1.0;
    joint_signs[9] = -1.0;
}

void MotionController::begin() {
    LittleFS.begin(true); // format if failed
}

int MotionController::mapPlen2ToJoint(const char* device) {
    String dev = String(device);
    if(dev == "left_shoulder_pitch") return J_SHOULDER_L;
    if(dev == "left_shoulder_roll") return J_HIGH_ARM_L;
    if(dev == "left_elbow_roll") return J_LOW_ARM_L;
    
    if(dev == "right_shoulder_pitch") return J_SHOULDER_R;
    if(dev == "right_shoulder_roll") return J_HIGH_ARM_R;
    if(dev == "right_elbow_roll") return J_LOW_ARM_R;
    
    if(dev == "left_thigh_roll") return J_PELVIS_L;
    if(dev == "left_thigh_pitch") return J_THIGH2_L;
    if(dev == "left_knee_pitch") return J_TIBIA_L;
    if(dev == "left_foot_pitch") return J_ANKLE1_L;
    if(dev == "left_foot_roll") return J_ANKLE2_L;
    
    if(dev == "right_thigh_roll") return J_PELVIS_R;
    if(dev == "right_thigh_pitch") return J_THIGH2_R;
    if(dev == "right_knee_pitch") return J_TIBIA_R;
    if(dev == "right_foot_pitch") return J_ANKLE1_R;
    if(dev == "right_foot_roll") return J_ANKLE2_R;
    
    return -1;
}

void MotionController::interpolatePose(uint8_t* start_pose, uint8_t* end_pose, int duration_ms, int torque) {
    int step_time_ms = 20;
    int steps = duration_ms / step_time_ms;
    if(steps < 1) steps = 1;
    
    for(int step = 1; step <= steps; step++) {
        uint8_t interpolated[NUM_JOINTS];
        float ratio = (float)step / steps;
        for(int i = 0; i < NUM_JOINTS; i++) {
            int val = start_pose[i] + (end_pose[i] - start_pose[i]) * ratio;
            if (val < 1) val = 1;
            if (val > 254) val = 254;
            interpolated[i] = val;
        }
        _wck->posGroup(15, torque, interpolated);
        delay(step_time_ms);
    }
    _wck->posGroup(15, torque, end_pose);
}

void MotionController::playMotion(const char* filepath, float speed, int torque) {
    File file = LittleFS.open(filepath, "r");
    if(!file) {
        Serial.println("Failed to open motion file");
        return;
    }
    
    // Allocate a temporary JsonDocument
    DynamicJsonDocument doc(8192); // Adjust size if needed
    DeserializationError error = deserializeJson(doc, file);
    if(error) {
        Serial.print("Failed to parse JSON: ");
        Serial.println(error.c_str());
        file.close();
        return;
    }
    file.close();
    
    JsonArray frames = doc["frames"];
    for(JsonObject frame : frames) {
        uint8_t target_pose[NUM_JOINTS];
        for(int i=0; i<NUM_JOINTS; i++) target_pose[i] = zero_offsets[i];
        
        int transition_time = frame["transition_time_ms"] | 100;
        JsonArray outputs = frame["outputs"];
        for(JsonObject output : outputs) {
            const char* device = output["device"];
            int value = output["value"];
            int joint_id = mapPlen2ToJoint(device);
            if(joint_id != -1) {
                float offset = value * value_scale;
                int pos = zero_offsets[joint_id] + joint_signs[joint_id] * offset;
                if(pos < 1) pos = 1;
                if(pos > 254) pos = 254;
                target_pose[joint_id] = pos;
            }
        }
        
        int adjusted_time = transition_time / speed;
        interpolatePose(cur_pose, target_pose, adjusted_time, torque);
        for(int i=0; i<NUM_JOINTS; i++) cur_pose[i] = target_pose[i];
    }
}

void MotionController::gotoZero() {
    _wck->posGroup(15, 4, zero_offsets);
    for(int i=0; i<NUM_JOINTS; i++) cur_pose[i] = zero_offsets[i];
    delay(500);
}

void MotionController::setPassiveMode() {
    for(int i=0; i<NUM_JOINTS; i++) {
        _wck->passivate(i);
        delay(10);
    }
    Serial.println("Motors are passive.");
}

void MotionController::printCurrentPose() {
    Serial.println("Current Pose (PLEN2 format):");
    Serial.print("{\"transition_time_ms\": 500, \"outputs\": [");
    bool first = true;
    for(int i=0; i<NUM_JOINTS; i++) {
        int raw_pos = _wck->readPos(i);
        if(raw_pos < 0) raw_pos = zero_offsets[i];
        
        float plen2_val = ((raw_pos - zero_offsets[i]) * joint_signs[i]) / value_scale;
        
        // Find device name (reverse mapping)
        const char* dev = nullptr;
        if(i == J_SHOULDER_L) dev = "left_shoulder_pitch";
        else if(i == J_HIGH_ARM_L) dev = "left_shoulder_roll";
        else if(i == J_LOW_ARM_L) dev = "left_elbow_roll";
        else if(i == J_SHOULDER_R) dev = "right_shoulder_pitch";
        else if(i == J_HIGH_ARM_R) dev = "right_shoulder_roll";
        else if(i == J_LOW_ARM_R) dev = "right_elbow_roll";
        else if(i == J_PELVIS_L) dev = "left_thigh_roll";
        else if(i == J_THIGH2_L) dev = "left_thigh_pitch";
        else if(i == J_TIBIA_L) dev = "left_knee_pitch";
        else if(i == J_ANKLE1_L) dev = "left_foot_pitch";
        else if(i == J_ANKLE2_L) dev = "left_foot_roll";
        else if(i == J_PELVIS_R) dev = "right_thigh_roll";
        else if(i == J_THIGH2_R) dev = "right_thigh_pitch";
        else if(i == J_TIBIA_R) dev = "right_knee_pitch";
        else if(i == J_ANKLE1_R) dev = "right_foot_pitch";
        else if(i == J_ANKLE2_R) dev = "right_foot_roll";
        
        if(dev) {
            if(!first) Serial.print(", ");
            Serial.print("{\"device\": \""); Serial.print(dev);
            Serial.print("\", \"value\": "); Serial.print((int)plen2_val);
            Serial.print("}");
            first = false;
        }
    }
    Serial.println("]}");
}
