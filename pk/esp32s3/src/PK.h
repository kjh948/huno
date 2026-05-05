#ifndef PK_H
#define PK_H

#include "WCK.h"
#include "Config.h"

class PK {
public:
    PK(WCK* wck);
    void begin();
    void startWalk();
    void stopWalk();
    void update(); // call in loop

    void printParams();
    void toggleSign(int joint_id);

    float f = 8.0;
    float robot_height = 0.5;
    float shift_y = 0.2;
    float step_height = 0.4;
    float step_length = 0.2;
    float arm_swing = 1.0;
    float direction = -1.0;
    float side_step = 0.0;
    float scale = 30.0;
    
    bool is_walking = false;

private:
    WCK* _wck;
    uint8_t targets[NUM_JOINTS];
    float signs[NUM_JOINTS];
    bool was_walking = false;
    
    void set_joint_val(int joint_id, float val_offset);
    void left_leg(float x, float y, float z);
    void right_leg(float x, float y, float z);
    void update_arms(float xLeft, float xRight);
    void interpolate_pose(uint8_t* start_pose, uint8_t* end_pose, int duration_ms, int torque=4);
};

#endif
