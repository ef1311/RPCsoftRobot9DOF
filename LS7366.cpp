#include "mbed.h"
#include "LS7366.h"

using namespace mbed;

LS7366::LS7366(PinName mosi, PinName miso, PinName sclk, PinName cs) : _spi(mosi, miso, sclk), _cs(cs) {
    
    _cs=1;
    // Set up the SPI for 16-bit values (12-bit + 4 command bits) and mode 0.
    _spi.format(8, 0);
    // Set SPI clock frequency in Hz.
    _spi.frequency(4000000);
    
    
    // Select the device by seting chip select low and clear the counter
    _cs = 0;
    _spi.write(0x20);  //32 dec     CLEAR counter
    _cs = 1; 

    // Select the device by seting chip select low and clear the status    
    _cs = 0;
    _spi.write(0x30);  //48 dec     CLEAR status
    _cs = 1; 
    
    // Select the device by seting chip select low and write mode0  
    _cs = 0;
    _spi.write(0x88);    //136 dec  wirte MODE 0
    //spi.write((char)B00000011);
    _spi.write(0x03);    //3 dec  (4 counts per cycle)
    _cs = 1; 
    
    _cs = 0;
    _spi.write(0x90);    //144 dec  write MODE1
    //spi.write((char)B00000010);
    _spi.write(0x02);    //2 dec    (2 bytes counter)
    _cs = 1;    
    return;
}

int LS7366::encodercount(){
    
    _cs = 0;
    int countbuf=0;
    _spi.write(0x60);    //96 dec   READ counter
    for(int i=2;i>0;i--){
    int data=_spi.write(0x00);
    countbuf=countbuf*255+data;}
    int count=countbuf;
    _cs = 1;
    // Deselect the device
    return count;}
    
void LS7366::initialize(){
    // Select the device by seting chip select low and clear the counter
    _cs = 0;
    _spi.write(0x20);  //32 dec
    _cs = 1; 

    // Select the device by seting chip select low and clear the status    
    _cs = 0;
    _spi.write(0x30);  //48 dec
    _cs = 1; 
}

void LS7366::hold(){
    // Select the device by seting chip select low and clear the counter
    _cs = 0;
    _spi.write(0x90);    //144 dec  write MODE1
    //spi.write((char)B00000010);
    _spi.write(0x06);    //2 dec    (2 bytes counter)
    _cs = 1;  
}

void LS7366::start(){
    // Select the device by seting chip select low and clear the counter
    _cs = 0;
    _spi.write(0x90);    //144 dec  write MODE1
    //spi.write((char)B00000010);
    _spi.write(0x02);    //2 dec    (2 bytes counter)
    _cs = 1;  
}


