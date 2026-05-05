#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

const int NUM_JOINTS = 16;
extern uint8_t zero_offsets[NUM_JOINTS];

// Joint Indices
enum JointID {
    J_PELVIS_L = 0,
    J_THIGH2_L = 1,
    J_TIBIA_L = 2,
    J_ANKLE1_L = 3,
    J_ANKLE2_L = 4,
    J_PELVIS_R = 5,
    J_THIGH2_R = 6,
    J_TIBIA_R = 7,
    J_ANKLE1_R = 8,
    J_ANKLE2_R = 9,
    J_SHOULDER_L = 10,
    J_HIGH_ARM_L = 11,
    J_LOW_ARM_L = 12,
    J_SHOULDER_R = 13,
    J_HIGH_ARM_R = 14,
    J_LOW_ARM_R = 15
};

#endif
