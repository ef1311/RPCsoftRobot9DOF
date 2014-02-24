#include "mbed.h"
#include "MCP4922.h"
#include "LS7366.h"
#include "Trajectory.h"
#include <string>
#include <sstream>
 
using namespace mbed;
MCP4922 MCP1(p5, p7,p8);  // MOSI, SCLK, CS
MCP4922 MCP2(p5, p7,p9);  // MOSI, SCLK, CS

Serial com(USBTX, USBRX);

LS7366 LS3(p5, p6, p7,p15);  // MOSI,MISO, SCLK, CS
AnalogOut Valve5(p18);

char str2[300]="";

int j=0;
int k=0;
int z=0;
int count3=0;

int cmd1;
int cmd2;
int cmd3;
int cmd4;
DigitalOut lock(p23);

Timer t3;
Timer t4;
Timer t5;
Timer t6;
int timecount=0;
float looptime1=1;
float looptime2=1;
float looptime3=1;
float looptime4=1;
float looptime5=1;
int reset=1;


//cylinder 3
float preval3=0;
float setpoint3=0;
float distance3=0;
float distance33=0;
float mindistance3=0;
float error3=0;
float speed3=0;
float smc3=0;
float offsetV3=1.27;
float set3=1;
float flag3=0;
float fspeed3=0;
float speed3prev=0;
float fspeed3prev=0;
float v3[2]={0};  //filter the speed
float w3[2]={0};
float va3[2]={0};  //filter the accelearation
float wa3[2]={0};
float setpointprev3=0;
float setpointprev33=0;
int manual3=0;
float setpoint333=0;
float setpoint33=0;
float aset3=0;
float vset3=0;


float T=1;
float time1=0;
float time2=0;
float time3=0;
Trajectory traj3(1);
Trajectory traj03(1);
int reset2=0;
int reset1=0;
int reset0=0;
float divfact=4;
int manual=0;
int frontpress=0;

float setpoint3330=0;
float aset30=0;
float vset30=0;
float setpointprev330=0;

float set3prev=0;
float set3prev3=0;
float set3e=offsetV3/3.3;
float set3eprev=offsetV3/3.3;
float locked=0;
float a3prev=0;
float a3prevf=0;
float k3=50;
float set3prev2=0;
float lambda=12.5;

float filt1=1;
float filt2=1;

int waittime=1;
float m=1;
float sinusoidalwave=0;
int trackingtest=0;

float saturation(float s, float p){
       if(s/p>1){return 1;}
       else if(s/p<-1){return -1;}
       else{return s/p;}
       }  
          
    int main() {

    com.baud(921600);

    MCP1.frequency(4000000);
    MCP2.frequency(4000000);

    t3.start();
    t4.start();
    t5.start();
    t6.start();
    lock=1;
    cmd3=0;//(int)4096*2.3/3.3;
    MCP2.writeA(cmd3);      //set constant pressure
    
    cmd1 = 0;//drive all cylinders in before starting counting
    MCP1.writeA(cmd1);
    cmd2 = 0;
    MCP1.writeB(cmd2);
    cmd4 = 0.3;
    MCP2.writeB(cmd4);
    
    Valve5=1;  //max pressure available is 2bar
    
    wait(5);
        
    LS3.initialize();
    
    setpoint3=1;
    
    //TEST SETUP
    waittime=800; //0; 100; 270; 450; 800; 1200;
    trackingtest=0;
    
    if(waittime==0){    //4.5kHz
        m=5;
        filt1=0.0730;
        filt2=0.8541;}
    if(waittime==100){  //3kHz
        m=4;
        filt1=0.1122;
        filt2=0.7757;}
    if(waittime==270){  //2kHz
        m=3;
        filt1=0.1602;
        filt2=0.6796;} 
    if(waittime==450){  //1.5kHz
        m=2;
        filt1=0.2043;
        filt2=0.5914;}
    if(waittime==800){  //1kHz
        m=1;
        filt1=0.2836;
        filt2=0.4327;}  
    if(waittime==1200){  //0.7kHz
        m=1;
        filt1=0.3716;
        filt2=0.2568;}  
    if(waittime==1400){  //0.6kHz
        m=1;
        filt1=0.4208;
        filt2=0.1584;}  

    //setpoint3=10;
    //sinusoidalwave=1;
    //T=5;
    
        while(1) {
        t3.reset();    
        t6.reset();     
        timecount=t5.read_us();
        count3=LS3.encodercount();
        j++;
        z++;

        if(trackingtest==1){
        //Tracking test
        if(k>10){
            sprintf(str2,"%4.3f//",looptime1);
            com.printf(str2);
            t5.reset();
            break;}
        if(timecount>2500000*(k+1)){
            k++;
            reset2=1;
            T=2.5;
            if(k%2==0){setpoint3=21;}
            else{setpoint3=1;}
            }  
        }
        else{
        //step test
        if(j>25000*m){  
            sprintf(str2,"%4.3f//",looptime1);
            com.printf(str2);
            reset2=1;
            T=0;
            setpoint3=1;
            j=0;
            t5.reset();
            break;}  
        if(j==100*m){
            reset2=1;
            T=0;
            setpoint3=1;}  
        if(j==4000*m){
            reset2=1;
            T=0;
            setpoint3=1.5;}
        if(j==6000*m){
            reset2=1;
            T=0;
            setpoint3=1;}    
        if(j==8000*m){
            reset2=1;
            T=0;
            setpoint3=2;}
        if(j==10000*m){
            reset2=1;
            T=0;
            setpoint3=1;}
        if(j==12000*m){
            reset2=1;
            T=0;
            setpoint3=6;}
        if(j==14000*m){
            reset2=1;
            T=0;
            setpoint3=1;}  
        if(j==16000*m){
            reset2=1;
            T=0;
            setpoint3=11;}
        if(j==18000*m){
            reset2=1;
            T=0;
            setpoint3=1;}
        if(j==20000*m){
            reset2=1;
            T=0;
            setpoint3=21;}
        if(j==22000*m){
            reset2=1;
            T=0;
            setpoint3=1;}                            
        }
        
        if(count3>5000){
        LS3.initialize();}
  
        //VALVE 3
        if(setpoint3<0){setpoint3=0;}
        if(setpoint3>80){setpoint3=80;}
        distance3=count3*25.4/(300*divfact);  //HEDS-9200-300 has 300 CPI
        
        if(reset2==1){
        t4.reset();
        distance33=distance3;
        setpointprev330=setpoint3330;
        reset2=0;}
        time3=t4.read();
        traj3.path(setpoint333, setpoint3, aset3, vset3, T, distance33, time3,sinusoidalwave);
        traj03.path(setpoint3330, setpoint3, aset30, vset30, T, setpointprev330, time3,sinusoidalwave);
        
        looptime2=t3.read_us()/1000.0;
        t3.reset();
        
        error3=setpoint333-distance3;
        
        speed3=(distance3-preval3)/(looptime1); 
        
        v3[0]=v3[1];
        v3[1]=filt1*speed3;//1367
        w3[0]=w3[1];
        w3[1]=v3[0]+v3[1]+filt2*w3[0];//7265
        fspeed3=w3[1];

        a3prevf=(fspeed3-fspeed3prev)/(looptime1/1000);
        
        va3[0]=va3[1];
        va3[1]=filt1*a3prevf;
        wa3[0]=wa3[1];
        wa3[1]=va3[0]+va3[1]+filt2*wa3[0];
        a3prev=wa3[1];     
        
        looptime3=t3.read_us()/1000.0;
        t3.reset();
        
        if(T==0){
        vset3=0;
        aset3=0;}
        lambda=12.5;
        smc3=lambda*error3+(vset3/1000-fspeed3)*1000;
        if(abs(smc3)>0.3){k3=20;}
        else{k3=0;}
        set3e=set3eprev+(0.3*(smc3*k3/1000+lambda*(vset3/1000-fspeed3)+(aset3/1000-a3prev))/80)/3.3;
        //set3e=set3e+0.0002*saturation(smc3/1000,0.25/(lambda*20))/3.3;
        
        if(set3e<0.3/3.3){set3e=0.3/3.3;}
        if(set3e>2.25/3.3){set3e=2.25/3.3;}
        
        set3=set3e+0.10*saturation(smc3/1000,0.25/(lambda*20))/3.3;

        if(set3<0.3/3.3){set3=0.3/3.3;}
        if(set3>2.25/3.3){set3=2.25/3.3;}
        
        set3eprev=set3e;

        looptime4=t3.read_us()/1000.0;
        t3.reset();
        
        //flag when error>limit
        if(abs(error3)>0.5){flag3=1;}
        else {flag3=0;}  
        
        //Write to DAC         
        cmd1 = (int)4096*0;
        MCP1.writeA(cmd1);
        
        cmd2 = (int)4096*0;
        MCP1.writeB(cmd2);
        
        cmd4 = (int)4096*set3;
        MCP2.writeB(cmd4);

        cmd3=0;
        MCP2.writeA(cmd3);     

        if(locked==0){
        lock=1;
        LS3.start();}
        else{
        lock=0;
        LS3.hold();}
 
        if(distance3<mindistance3){mindistance3=distance3;}      

        wait_us(waittime);

        preval3=distance3;

        setpointprev3=setpoint3;
        setpointprev33=setpoint33;
        
        set3prev3=set3prev2;
        set3prev2=set3;
        
        looptime5=t3.read_us()/1000.0;
        t3.reset();

        if(z==1){sprintf(str2,"%1d/",timecount);} 
        if(z==2){sprintf(str2,"%4.3f/",distance3);}
        if(z==3){sprintf(str2,"%4.3f/",setpoint333);
        z=0;}            

        //sprintf(str2,"%4.3f/",looptime1);
        //sprintf(str2,"%4.3f/%4.3f/",looptime1,looptime5);
        //sprintf(str2,"%4.3f/%4.3f/%4.3f/%4.3f/%4.3f/",looptime1,looptime2,looptime3,looptime4,looptime5);
        //sprintf(str2,"%4.3f/%1d/%4.3f/%4.3f/",distance3,timecount,setpoint333,set3*3.3);
        //sprintf(str2,"%4.3f/%1d/%4.3f/",distance3,timecount,setpoint333);
        //sprintf(str2,"%4.3f/%1d/%4.3f/%4.3f/%4.3f/",speed3,timecount,fspeed3,a3prev,a3prevf);
        com.printf(str2);

        looptime1=t6.read_us()/1000.0;
        fspeed3prev=fspeed3;
        speed3prev=speed3;
        
        
        }
        }