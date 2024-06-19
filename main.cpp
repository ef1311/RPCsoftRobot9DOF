#include "mbed.h"
#include "SerialRPCInterface.h"
#include "MCP4922.h"

// mbed code for digital pressure regulators

using namespace mbed;
MCP4922 MCP1(p5, p7,p15);  // MOSI, SCLK, CS
MCP4922 MCP2(p5, p7,p16);  // MOSI, SCLK, CS
MCP4922 MCP3(p5, p7,p17);  // MOSI, SCLK, CS
MCP4922 MCP4(p5, p7,p19);  // MOSI, SCLK, CS
MCP4922 MCP5(p5, p7,p20);  // MOSI, SCLK, CS
SerialRPCInterface RPC(USBTX, USBRX, 921600); 

int cmd1=0;
int cmd2=0;
int cmd3=0;
int cmd4=0;
int cmd5=0;
int cmd6=0;
int cmd7=0;
int cmd8=0;
int cmd9=0;
int cmd10=0;

//pressures for valves 1-10
float set1=0.0;
float set2=0.0;
float set3=0.0;
float set4=0.0;
float set5=0.0;
float set6=0.0;
float set7=0.0;
float set8=0.0;
float set9=0.0;
float set10=0.0;        
          
    int main() {
    RPCVariable<float> rpc_f(&set1, "set1");
    RPCVariable<float> rpc_e(&set2, "set2");
    RPCVariable<float> rpc_d(&set3, "set3");
    RPCVariable<float> rpc_g(&set4, "set4");
    RPCVariable<float> rpc_h(&set5, "set5");
    RPCVariable<float> rpc_k(&set6, "set6");
    RPCVariable<float> rpc_l(&set7, "set7");
    RPCVariable<float> rpc_m(&set8, "set8");
    RPCVariable<float> rpc_n(&set9, "set9");
    RPCVariable<float> rpc_p(&set10, "set10");

    MCP1.frequency(4000000);
    MCP2.frequency(4000000);
    
       
    MCP1.writeA(cmd1);   // set all pressures to zero
    MCP1.writeB(cmd2);
    MCP2.writeA(cmd3);  
    MCP2.writeB(cmd4);   
    MCP3.writeA(cmd5);   // set all pressures to zero
    MCP3.writeB(cmd6);
    MCP4.writeA(cmd7);  
    MCP4.writeB(cmd8);
    MCP5.writeA(cmd9);  
    MCP5.writeB(cmd10);
   
    wait(1);  
    
        while(1) {
        cmd1 =(int)4096*set1/3.3;   // scaling factor 3.3 for valves 1-10
        MCP1.writeA(cmd1);
        cmd2 = (int)4096*set2/3.3;
        MCP1.writeB(cmd2);
        cmd3 = (int)4096*set3/3.3;
        MCP2.writeA(cmd3);  
        cmd4 = (int)4096*set4/3.3;
        MCP2.writeB(cmd4);
        cmd5 = (int)4096*set5/3.3;   
        MCP3.writeA(cmd5);
        cmd6 = (int)4096*set6/3.3;
        MCP3.writeB(cmd6);
        cmd7 = (int)4096*set7/3.3;
        MCP4.writeA(cmd7);  
        cmd8 = (int)4096*set8/3.3;
        MCP4.writeB(cmd8);
        cmd9 = (int)4096*set9/3.3;
        MCP5.writeA(cmd9);  
        cmd10 = (int)4096*set10/3.3;
        MCP5.writeB(cmd10);
        wait_us(500);       //nominal loop frequency 2 kHz (to be verified)
        }
        }
        
