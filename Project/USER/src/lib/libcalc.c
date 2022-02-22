#include "headfile.h"
#include "string.h"

#define LOST_FLAG 255

uint8 maxInt(uint8 a,uint8 b){
    return a > b ? a : b;
}

uint8 minInt(uint8 a,uint8 b){
    return a > b ? b : a;
}

int16 fix(float x){
    //将数字舍入最近的整数
    float floorDiff,ceilDiff;
    int16 xInt=(int16)x;
    if(x==xInt){
        return xInt;
    }
    else if(x > 0){
        floorDiff = x-xInt;
        ceilDiff = xInt+1-x;
        if(floorDiff>ceilDiff){
            return xInt+1; 
        }
        else{
            return xInt;
        }
    }
    else if(x < 0){
        floorDiff = xInt-x;
        ceilDiff = x-xInt+1;
        if(floorDiff>ceilDiff){
            return xInt; 
        }
        else{
            return xInt-1;
        }
    }
}

float averagey(uint8 line[MT9V03X_CSI_H],uint8 *n,uint8 startLine,uint8 endLine){
    uint16 sum=0;
    float avg=0;
    uint8 countVaild=0;
    for(uint8 i=endLine;i<startLine;i++){
        if(line[i]!=LOST_FLAG){
            sum+=i;
            countVaild++;
        }
    }
    avg=(float)sum/countVaild;
    *n=countVaild;
    return avg;
}

float averagex(uint8 line[MT9V03X_CSI_H],uint8 startLine,uint8 endLine){
    uint16 sum=0;
    float avg=0;
    uint8 countVaild=0;
    for(uint8 i=endLine;i<startLine;i++){
        if(line[i]!=LOST_FLAG){
            sum+=line[i];
            countVaild++;
        }
    }
    avg=(float)sum/countVaild;
    return avg;
}

uint16 sumxy(uint8 line[MT9V03X_CSI_H],uint8 startLine,uint8 endLine){
    uint16 sum=0;
    for(uint8 i=endLine;i<startLine;i++){
        if(line[i]!=LOST_FLAG){
            sum+=line[i]*i;
        }
    }
    return sum;
}

uint16 sumxSquare(uint8 line[MT9V03X_CSI_H],uint8 startLine,uint8 endLine){
    uint16 sum=0;
    for(uint8 i=endLine;i<startLine;i++){
        if(line[i]!=LOST_FLAG){
            sum+=line[i]*line[i];
        }
    }
    return sum;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      直线拟合
//  @param      line[] 直线数组 uint8型 丢线设为LOST_FLAG 
//  @param      lineFitted[] 直线数组 int16型 考虑到左侧丢线拟合后出现负数的情况
//  @return     float 返回拟合后的斜率					
//-------------------------------------------------------------------------------------------------------------------

float linearFitting(uint8 line[MT9V03X_CSI_H],int16 lineFitted[MT9V03X_CSI_H],uint8 startLine,uint8 endLine){
    uint8 countVaild=0;
    float xAvg,yAvg,slope,aHat,bHat,c,d;
    uint16 xySum,xSquareSum;

    memset(lineFitted,0,sizeof(lineFitted[0]*MT9V03X_CSI_H)); //处理范围以外的部分置FF

    startLine=minInt(startLine,MT9V03X_CSI_H);
    endLine=maxInt(startLine,0);

    yAvg=averagey(line,&countVaild,startLine,endLine);
    xAvg=averagex(line,startLine,endLine);
    xySum=sumxy(line,startLine,endLine);
    xSquareSum=sumxSquare(line,startLine,endLine);
    //计算最小二乘法的b和a
    bHat=(float)(xySum-countVaild*xAvg*yAvg)/(xSquareSum-countVaild*xAvg*xAvg);
    aHat=yAvg-bHat*xAvg;
    //反解y=bx+a => x=y/b-a/b x=cy+d可由y给出拟合后直线x
    c=1/bHat;
    d=-aHat/bHat;
    //根据c与d在lineFitted中补线
    for(uint8 i=endLine;i<startLine;i++){
        lineFitted[i]=fix(c*i+d);
    }
    return bHat;
}