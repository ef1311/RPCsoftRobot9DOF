#include "mbed.h"
#include "MCP4922.h"
#include "LS7366.h"
#include "SerialRPCInterface.h"
 
using namespace mbed;
MCP4922 MCP1(p5, p7,p15);  // MOSI, SCLK, CS
MCP4922 MCP2(p5, p7,p16);  // MOSI, SCLK, CS
SerialRPCInterface RPC(USBTX, USBRX, 921600); 

LS7366 LS2(p5, p6, p7,p13);  // MOSI,MISO, SCLK, CS
LS7366 LS1(p5, p6, p7,p14);  // MOSI,MISO, SCLK, CS

AnalogIn press1(p18);
AnalogIn press2(p20);

Timer t1;
float looptime1=1;

int cmd1=0;
int cmd2=0;

//pressures for valves 1-2
float set1=0.0;
float set2=0.0;
float offsetV2=2.3/3.3;
float offsetV1=1.22/3.3;

// force sensor reading
float force=0;
float forcev[50]={0};
float force0=0;
float v3[2]={0};  //filter the force
float w3[2]={0};

// encoder readings
int count1=0;
int count2=0;
float distance1=0;
float distance2=0;
float preval1=0;
float preval2=0;
int reset0=0;
float divfact=4;

// butterworth low pass filter for speed
float v1[2]={0};  //filter the speed
float w1[2]={0};
float v2[2]={0};  //filter the speed
float w2[2]={0};
float filt1=0.1602;
float filt2=0.6796;
float speed1=0;
float speed2=0;
float fspeed1=0;
float fspeed2=0;
                     
    int main() {
    
    RPCVariable<float> rpc_f(&set1, "set1");
    RPCVariable<float> rpc_e(&set2, "set2");
    RPCVariable<float> rpc_d(&distance1, "distance1");
    RPCVariable<float> rpc_g(&distance2, "distance2");
    RPCVariable<float> rpc_h(&force, "force");
    RPCVariable<float> rpc_i(&fspeed1, "fspeed1");
    RPCVariable<float> rpc_j(&fspeed2, "fspeed2");

    MCP1.frequency(4000000);
    MCP2.frequency(4000000);
    
    cmd1 = (int)4096*0.5/3.3;
    MCP1.writeA(cmd1);
    cmd2 = (int)4096*2.3/3.3;
    MCP1.writeB(cmd2);
    
    wait(2);
        
    LS1.initialize();
    LS2.initialize();
    
    for(int i=0;i<50;i++){
    forcev[i]=press1-press2;
    force0=force0+forcev[i]/50;}
    
        while(1) {
        
        // encoder position
        count1=LS1.encodercount();
        count2=LS2.encodercount();
        
        if(count1>5000){
        LS1.initialize();}
        if(count2>5000){
        LS2.initialize();}
        
        distance1=count1*25.4/(250*divfact);
        distance2=count2*25.4/(250*divfact);
        
        // velocity
        speed1=(distance1-preval1)/(looptime1);
        v1[0]=v1[1];
        v1[1]=filt1*speed1;
        w1[0]=w1[1];
        w1[1]=v1[0]+v1[1]+filt2*w1[0];
        fspeed1=w1[1];
        
        speed2=(distance2-preval2)/(looptime1);
        v2[0]=v2[1];
        v2[1]=filt1*speed2;
        w2[0]=w2[1];
        w2[1]=v2[0]+v2[1]+filt2*w2[0];
        fspeed2=w2[1];
        speed2=(distance2-preval2)/(looptime1);
           
        // force sensor   
        v3[0]=v3[1];
        v3[1]=0.01547*(press1-press2);    
        w3[0]=w3[1];
        w3[1]=v3[0]+v3[1]+0.9691*w3[0];
        force=(w1[1]-force0)*10;
        if(force<0){force=0;}
        
        // pressure regulators
        cmd1 =(int)4096*set1/3.3;   // scaling factor 3.3
        MCP1.writeA(cmd1);
        cmd2 = (int)4096*set2/3.3;
        MCP1.writeB(cmd2);
        
        wait_us(500);   //nominal loop frequency 2 kHz (to be verified)
        
        preval1=distance1;
        preval2=distance2;
        looptime1=t1.read_us()/1000.0;
        
        }
        }