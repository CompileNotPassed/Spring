#include "headfile.h"
#define maxspeed 100
void mecmove(uint8 vx,uint8 vy,uint8 va){
    float vMotor[4],max;
    
    vMotor[0]=vy+vx-va;
    vMotor[1]=vy-vx+va;
    vMotor[2]=vy-vx-va;
    vMotor[3]=vy+vx+va;
    
    max=vMotor[0];
    for(char i=1;i<4;i++){
        max=max>vMotor[i]?max:vMotor[i];
    }

    if(max>maxspeed){
        for(char i=0;i<4;i++){
            vMotor[i]=vMotor[i]/max*maxspeed;
        }
    }

}