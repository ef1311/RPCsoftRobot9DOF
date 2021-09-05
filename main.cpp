#include "mbed.h"
#include "SerialRPCInterface.h"
#include "MCP4922.h"

using namespace mbed;
MCP4922 MCP1(p5, p7,p8);  // MOSI, SCLK, CS
MCP4922 MCP2(p5, p7,p9);  // MOSI, SCLK, CS
SerialRPCInterface RPC(USBTX, USBRX, 921600); 

AnalogOut Valve5(p18);
DigitalOut lock(p23);

int cmd1=0;
int cmd2=0;
int cmd3=0;
int cmd4=0;

//pressures for valves 1-5
float set1=0.0;
float set2=0.0;
float set3=0.0;
float set4=0.0;
float set5=0.0;
        
          
    int main() {
    RPCVariable<float> rpc_f(&set1, "set1");
    RPCVariable<float> rpc_e(&set2, "set2");
    RPCVariable<float> rpc_d(&set3, "set3");
    RPCVariable<float> rpc_g(&set4, "set4");
    RPCVariable<float> rpc_h(&set5, "set5");

    MCP1.frequency(4000000);
    MCP2.frequency(4000000);
      
    lock=1;     // closes port "!"
       
    MCP1.writeA(cmd1);   // set all pressures to zero
    MCP1.writeB(cmd2);
    MCP2.writeA(cmd3);  
    MCP2.writeB(cmd4);   
    Valve5=set5/2; 
   
    wait(1);  
    
        while(1) {
        cmd1 =(int)4096*set1/3.3;   // scaling factor 3.3 for valves 1-4
        MCP1.writeA(cmd1);
        cmd2 = (int)4096*set2/3.3;
        MCP1.writeB(cmd2);
        cmd3 = (int)4096*set3/3.3;
        MCP2.writeA(cmd3);  
        cmd4 = (int)4096*set4/3.3;
        MCP2.writeB(cmd4);
        
        Valve5=set5/2;  // scaling factor 2 for valve 5
        lock=1;
        
        wait_us(500);       //nominal loop frequency 2 kHz (to be verified)
        }
        }
        
