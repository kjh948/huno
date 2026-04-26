#ifndef MOTION_H
#define MOTION_H

#include "WCK.h"
#include "Config.h"

class MotionController {
public:
    MotionController(WCK* wck);
    void begin();
    
    // Play motion from LittleFS file
    void playMotion(const char* filepath, float speed = 1.0, int torque = 4);
    
    // Read current pose and generate a frame (for motion generator)
    void printCurrentPose();
    void setPassiveMode();
    void gotoZero();

private:
    WCK* _wck;
    uint8_t cur_pose[NUM_JOINTS];
    float joint_signs[NUM_JOINTS];
    float value_scale = 0.1;
    
    int mapPlen2ToJoint(const char* device);
    void interpolatePose(uint8_t* start_pose, uint8_t* end_pose, int duration_ms, int torque);
};

#endif
