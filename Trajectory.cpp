#include "Trajectory.h"
#include "mbed.h"


    float power(float s, int p){
       float val=1;
       if(p>1){
           for(int i=1;i<=p;i++){
              val=val*s;}
           return val;}
       else{return s;}
       }
       
     Trajectory::Trajectory(int n){
     }
     
     void Trajectory::path(float& setpoint22, float setpoint2, float& aset22, float& vset22, float T, float distance22, float time2, float sinuswave){//,float val1s, float& val1st, float instdist1){
     float Jm1=0;
     float t1=0;
     float pi=3.141;
            
        if(sinuswave==0){
         if(T>0){          
            Jm1=32*(setpoint2-distance22)/(T*T*T);
            if(time2<T/4){
                //val1st=val1s;//instdist1;
                aset22=Jm1*time2;
                vset22=Jm1*power(time2,2)/2;
                setpoint22=Jm1*power(time2,3)/6+distance22;}//+0.125*sin(628*time2);}
            else{
                if(time2<3*T/4){
                    //val1st=val1s;//instdist1;
                    t1=time2-T/4;
                    aset22=Jm1*T/4-Jm1*t1;
                    vset22=Jm1*power(T/4,2)/2+Jm1*t1*T/4-Jm1*power(t1,2)/2;
                    setpoint22=Jm1*(T*T*T/64)/6+Jm1*(T*T/16)/2*(t1)+Jm1*T*power(t1,2)/8-Jm1*power(t1,3)/6+distance22;}//+0.125*sin(628*time2);}
                else if (time2<T){
                    //val1st=val1s;//instdist1;
                    t1=time2-T*3/4;
                    aset22=-Jm1*T/4+Jm1*t1;
                    vset22=Jm1*power(T/4,2)/2-Jm1*t1*T/4+Jm1*power(t1,2)/2;
                    setpoint22=Jm1*(T*T*T/64)/6+Jm1*(T*T/16)/2*(T/2)+Jm1*T*(T*T/4)/8-Jm1*(T*T*T/8)/6+Jm1*(T*T/16)/2*t1-Jm1*T*power(t1,2)/8+Jm1*power(t1,3)/6+distance22;}//+0.125*sin(628*time2);}
                else{
                    //val1st=val1s;
                    aset22=0;
                    vset22=0;
                    setpoint22=setpoint2;}
                    }
                    }
          else{
            setpoint22=setpoint2;}
            }
        else{
        setpoint22=2*setpoint2+setpoint2*sin(2*pi*time2/T);
        vset22=2*pi*setpoint2*cos(2*pi*time2/T)/T;
        aset22=-4*pi*pi*setpoint2*sin(2*pi*time2/T)/(T*T);
            }
}