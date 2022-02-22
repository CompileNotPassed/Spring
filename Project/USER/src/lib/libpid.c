#include "function.h"

pid_t Motor[4];

void Position_PID_Init(pid_t *PID_Motor, float Kp, float Ki, float Kd, int maxintegral, int integrallimit)
{
    PID_Motor->SetValue = 0;
    PID_Motor->ActualValue = 0;
    PID_Motor->err = 0;
    PID_Motor->err_last = 0;
    PID_Motor->err_last_last = 0;
    PID_Motor->integral = 0;
    PID_Motor->Kp = Kp;
    PID_Motor->Ki = Ki;
    PID_Motor->Kd = Kd;
    PID_Motor->maxintegral = maxintegral;
    PID_Motor->integrallimit = integrallimit;
}

float Position_PID_Realize(pid_t *PID_Motor, float Target, float Input)
{
    PID_Motor->SetValue = Target;
    PID_Motor->ActualValue = Input;
    PID_Motor->err = PID_Motor->SetValue - PID_Motor->ActualValue;
    if (PID_Motor->err <= PID_Motor->integrallimit && PID_Motor->err >= -PID_Motor->integrallimit)
    {
        PID_Motor->integral += PID_Motor->err;
        PID_Motor->result = PID_Motor->Kp * PID_Motor->err + PID_Motor->Ki * PID_Motor->integral + PID_Motor->Kd * (PID_Motor->err - PID_Motor->err_last);
    }
    else
    {
        PID_Motor->result = PID_Motor->Kp * PID_Motor->err + PID_Motor->Kd * (PID_Motor->err - PID_Motor->err_last);
        PID_Motor->integral = 0;
    }
    if (PID_Motor->integral >= PID_Motor->maxintegral)
        PID_Motor->integral = PID_Motor->maxintegral;
    if (PID_Motor->integral <= -PID_Motor->maxintegral)
        PID_Motor->integral = -PID_Motor->maxintegral;
    PID_Motor->err_last = PID_Motor->err;
    return PID_Motor->result;
}

float absNum(float num)
{
    return num > 0 ? num : -num;
}

// integralLimit
uint8 beta(float err, float threshold)
{
    return absNum(err) > threshold ? 0 : 1;
}

float Position_PID_Implementation(pid_t *obj, float Target, float Input)
{
    if (obj->SetValue != Target)
    {
        obj->SetValue = Target;
    }
    obj->ActualValue = Input;
    // Update err
    obj->err_last = obj->err;
    obj->err = Target - Input;
    // maxIntegral
    uint8 delta = abs(obj->integral) - obj->maxintegral;
    if (delta > 0)
    {
        if (obj->integral > 0)
        {
            if (obj->err > 0 || (obj->err < 0 && obj->err > -delta))
            {
                obj->integral = obj->maxintegral;
            }
            else
            {
                obj->integral += obj->err;
            }
        }
        if (obj->integral < 0)
        {
            if (obj->err < 0 || (obj->err > 0 && obj->err < delta))
            {
                obj->integral = -obj->maxintegral;
            }
            else
            {
                obj->integral += obj->err;
            }
        }
    }
    //Output
    //obj->result = obj->Kp * obj->err + beta(obj->err, obj->integrallimit) * obj->Ki * obj->integral + obj->Kd * (obj->err - obj->err_last);
    obj->result = obj->Kp * obj->err + obj->Ki * obj->integral + obj->Kd * (obj->err - obj->err_last);
    // obj->result = obj->Kp * obj->err;
    return obj->result;

}

float PID_Implementation(pid_t *obj, float Target, float Input)
{
    float increment;
    obj->SetValue=Target;
    obj->ActualValue=Input;
    obj->err=obj->SetValue- obj->ActualValue;
    increment=obj->Kp*(obj->err - obj->err_last)+obj->Ki*obj->err+obj->Kd*(obj->err - 2*obj->err_last+obj->err_last_last);
    obj->err_last_last= obj->err_last;
    obj->err_last=obj->err;
    return increment;
}