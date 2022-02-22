// #include"headfile.h"
// #define L 'L'
// #define R 'R'

// //-------------------------------------------------------------------------------------------------------------------
// //  @brief      圆环识别
// //  @param      leftLine[] 直线数组 int16型 丢线设为LOST_FLAG 255
// //  @param      rightLine[] 直线数组 int16型 丢线设为LOST_FLAG 255
// //  @return     char 返回圆环方向					
// //-------------------------------------------------------------------------------------------------------------------


// char regRing(int16 leftLine[MT9V03X_CSI_H],int16 rightLine[MT9V03X_CSI_H]){
//     char direction=-1; //未检测到圆环为-1 两侧都有突变为-2
//     char ringFlag1,ringFlag2,ringFlag3;
//     char i,lDiff,rDiff;
//     //左侧判定
//     for(i=MT9V03X_CSI_H;i>1;i--){
//         lDiff=leftLine[i-1]-leftLine[i];
//         if(lDiff>3||lDiff<-3){
//             direction=R;
//             break;
//         }
//     }
//     //右侧判定
//     for(i=MT9V03X_CSI_H;i>1;i--){
//         if(rDiff>3||rDiff<-3){
//             if(direction==R){
//                 direction=-2;
//                 return direction;
//             }
//             break;
//         }
//     }
// }   