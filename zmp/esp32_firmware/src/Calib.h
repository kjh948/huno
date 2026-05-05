#ifndef CALIB_H
#define CALIB_H

#include "WCK.h"
#include "Config.h"

class Calib {
public:
    Calib(WCK* wck);
    void startCalibration(bool isWebSerial = false); // 캘리브레이션 모드 진입
    void processCalibInput(String input);             // 입력 1개 처리 (non-blocking)
    bool isCalibActive() { return _calibMode; }
    void loadZero();

private:
    WCK* _wck;
    bool _calibMode = false;
    int  _calibCurrentId = -1;
    bool _isWebSerial = false;
    void _print(String msg);
    void printJoints();
    void saveZero();
};

#endif
