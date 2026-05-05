#include "PK.h"
#include <math.h>

PK::PK(WCK* wck) : _wck(wck) {
    for(int i=0; i<NUM_JOINTS; i++) {
        targets[i] = zero_offsets[i];
        signs[i] = 1.0;
    }
    signs[J_ANKLE1_R] = -1.0;
    signs[J_TIBIA_L] = -1.0;
    signs[J_THIGH2_L] = -1.0;
}

void PK::begin() {
    _wck->posGroup(15, 4, zero_offsets);
}

void PK::startWalk() {
    is_walking = true;
}

void PK::stopWalk() {
    is_walking = false;
}

void PK::set_joint_val(int joint_id, float val_offset) {
    float offset = val_offset * scale * signs[joint_id];
    int target = zero_offsets[joint_id] + (int)offset;
    if (target < 1) target = 1;
    if (target > 254) target = 254;
    targets[joint_id] = target;
}

void PK::left_leg(float x, float y, float z) {
    set_joint_val(J_TIBIA_L, z);
    set_joint_val(J_THIGH2_L, -z/2.0 + x);
    set_joint_val(J_ANKLE1_L, -z/2.0 - x);
    set_joint_val(J_PELVIS_L, y);
    set_joint_val(J_ANKLE2_L, -y);
}

void PK::right_leg(float x, float y, float z) {
    set_joint_val(J_TIBIA_R, z);
    set_joint_val(J_THIGH2_R, -z/2.0 + x);
    set_joint_val(J_ANKLE1_R, -z/2.0 - x);
    set_joint_val(J_PELVIS_R, y);
    set_joint_val(J_ANKLE2_R, -y);
}

void PK::update_arms(float xLeft, float xRight) {
    set_joint_val(J_SHOULDER_L, arm_swing * xRight);
    set_joint_val(J_SHOULDER_R, arm_swing * xLeft);
}

void PK::printParams() {
    Serial.println("\n--- Current Parameters ---");
    Serial.print("  (f)  Frequency:    "); Serial.print(f);
    Serial.print("  (h)  Robot Height: "); Serial.println(robot_height);
    Serial.print("  (y)  Shift Y:      "); Serial.print(shift_y);
    Serial.print("  (sh) Step Height:  "); Serial.println(step_height);
    Serial.print("  (sl) Step Length:  "); Serial.print(step_length);
    Serial.print("  (ss) Side Step:    "); Serial.println(side_step);
    Serial.print("  (as) Arm Swing:    "); Serial.print(arm_swing);
    Serial.print("  (d)  Direction:    "); Serial.println(direction);
    Serial.print("  (sc) Servo Scale:  "); Serial.println(scale);
    Serial.print("  Status: "); Serial.println(is_walking ? "WALKING" : "STOPPED");
}

void PK::toggleSign(int joint_id) {
    if (joint_id >= 0 && joint_id < NUM_JOINTS) {
        signs[joint_id] *= -1.0;
        Serial.print("Sign of ID "); Serial.print(joint_id); Serial.print(" is now "); Serial.println(signs[joint_id]);
    }
}

void PK::interpolate_pose(uint8_t* start_pose, uint8_t* end_pose, int duration_ms, int torque) {
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

void PK::update() {
    if (is_walking) {
        float t = (millis() / 1000.0) * direction * f;
        
        float y_sway = sin(t) * shift_y;
        float y_left_step = cos(t) * side_step;
        float y_right_step = cos(t + PI) * side_step;
        
        float yLeft = y_sway + y_left_step;
        float yRight = y_sway + y_right_step;
        
        float zLeft = (sin(t) + 1.0) / 2.0 * step_height + robot_height;
        float zRight = (sin(t + PI) + 1.0) / 2.0 * step_height + robot_height;
        
        float xLeft = cos(t) * step_length;
        float xRight = cos(t + PI) * step_length;
        
        left_leg(xLeft, yLeft, zLeft);
        right_leg(xRight, yRight, zRight);
        update_arms(xLeft, xRight);
        
        _wck->posGroup(15, 4, targets);
        was_walking = true;
        delay(20);
    } else {
        if (was_walking) {
            interpolate_pose(targets, zero_offsets, 500);
            for(int i=0; i<NUM_JOINTS; i++) targets[i] = zero_offsets[i];
            was_walking = false;
        } else {
            _wck->posGroup(15, 4, zero_offsets);
            delay(100);
        }
    }
}
