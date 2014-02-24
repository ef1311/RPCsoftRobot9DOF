
#include "mbed.h"

#ifndef LS7366_H
#define LS7366_H

class LS7366 {
public:
     LS7366(PinName mosi, PinName miso, PinName sclk, PinName cs);
     int encodercount();
     void initialize();
     void hold();
     void start();
private:

     SPI _spi;
     DigitalOut _cs;
};
#endif