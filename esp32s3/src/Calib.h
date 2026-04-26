#ifndef CALIB_H
#define CALIB_H

#include "WCK.h"
#include "Config.h"

class Calib {
public:
    Calib(WCK* wck);
    void runCalibrationMenu();
    void loadZero();
    
private:
    WCK* _wck;
    void printJoints();
    void saveZero();
};

#endif
