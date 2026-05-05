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

// --- GankenKun ZMP Preview Control Variables ---
// These gains should be pre-calculated in Python using scipy.linalg.solve_discrete_are
const float dt = 0.01; 
const float z_c = 0.27; // CoM Height
const float A_d[3][3] = {{1.0, dt, dt*dt/2.0}, {0.0, 1.0, dt}, {0.0, 0.0, 1.0}};
const float B_d[3] = {dt*dt*dt/6.0, dt*dt/2.0, dt};
const float C_d[3] = {1.0, 0.0, -z_c / 9.81};

// Example pre-calculated Gains (Mock values, MUST be recalculated for exact robot)
const float F_gain[4] = {-200.0, -50.0, -10.0, -1.0};
// -------------------------------------------

float com_x[3] = {0,0,0}; // CoM State X
float com_y[3] = {0,0,0}; // CoM State Y

void PK::update() {
    if (is_walking) {
        // 1. ZMP Preview Control: Calculate Control Input (u)
        // Here we would typically multiply the error and future ZMP references with F and f_gain.
        float u_x = 0.0; // Mock control input
        float u_y = 0.0; 

        // 2. Update CoM State using State Space Model (x = Ax + Bu)
        float next_com_x[3], next_com_y[3];
        for (int i=0; i<3; i++) {
            next_com_x[i] = A_d[i][0]*com_x[0] + A_d[i][1]*com_x[1] + A_d[i][2]*com_x[2] + B_d[i]*u_x;
            next_com_y[i] = A_d[i][0]*com_y[0] + A_d[i][1]*com_y[1] + A_d[i][2]*com_y[2] + B_d[i]*u_y;
        }
        for (int i=0; i<3; i++) { com_x[i] = next_com_x[i]; com_y[i] = next_com_y[i]; }

        // 3. Foot Trajectory Generation (Mock implementation)
        // Here we generate the X, Y, Z of the swinging foot based on time 't'.
        float t = (millis() / 1000.0) * direction * f;
        float zLeft = (sin(t) + 1.0) / 2.0 * step_height + robot_height;
        float zRight = (sin(t + PI) + 1.0) / 2.0 * step_height + robot_height;
        float xLeft = com_x[0] + cos(t) * step_length; // Relative to CoM
        float xRight = com_x[0] + cos(t + PI) * step_length;
        float yLeft = com_y[0] + shift_y;
        float yRight = com_y[0] - shift_y;

        // 4. Inverse Kinematics
        // Use Analytical IK instead of simple joint offsets
        left_leg(xLeft, yLeft, zLeft);
        right_leg(xRight, yRight, zRight);
        update_arms(xLeft, xRight);
        
        // 5. Send to Motors
        _wck->posGroup(15, 4, targets);
        was_walking = true;
        // No delay here! Controlled by FreeRTOS vTaskDelayUntil in main.cpp
    } else {
        if (was_walking) {
            interpolate_pose(targets, zero_offsets, 500);
            for(int i=0; i<NUM_JOINTS; i++) targets[i] = zero_offsets[i];
            was_walking = false;
        } else {
            // Keep zero pose active
            // _wck->posGroup(15, 4, zero_offsets);
        }
    }
}
