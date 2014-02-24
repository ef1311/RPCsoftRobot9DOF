#ifndef MBED_TRAJECTORY_H
#define MBED_TRAJECTORY_H

#include "mbed.h"

class Trajectory {
public:
     Trajectory(int n);
     void path(float& setpoint22, float setpoint2,float& aset22, float& vset22, float T ,float distance22, float time2, float sinuswave);//, float val1s, float& val1st, float instdist2);
};
#endif