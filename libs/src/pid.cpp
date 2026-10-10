#include "pid.hpp"

PID::PID(float kp, float ki, float kd, float maxOut, float maxIOut, int mode)
    : mode(mode), kp(kp), ki(ki), kd(kd), maxOut(maxOut), maxIOut(maxIOut)
{
    Clear();
}

void PID::Tuning(float tuning_kp, float tuning_ki, float tuning_kd)
{
    kp = tuning_kp;
    ki = tuning_ki;
    kd = tuning_kd;
}

void PID::UpdateResult()
{
    err[2] = err[1];
    err[1] = err[0];
    err[0] = ref - fdb;

    const float oldIntegral = iResult;
    iResult=Numeric::LimitABS(iResult+err[0]*ki,std::fabs(maxIOut));
    if(mode==PID_POSITION)
    {
        pResult=kp*err[0];
        
        dResult=(err[0]-err[1])*kd;
        result=Numeric::LimitABS(pResult+iResult+dResult,std::fabs(maxOut));
    }
    else if(mode==PID_DELTA)
    {
        pResult=(err[0]-err[1])*kp;
        const float deltaIntegral = iResult - oldIntegral;
       
        dResult=kd*(err[0]-2.0f*err[1]+err[2]);
        result = Numeric::LimitABS(
            result + pResult + deltaIntegral + dResult,
            std::fabs(maxOut));



    }
    else
     {
        // 遇到无效模式时，安全地输出零
        pResult = 0.0f;
        iResult = 0.0f;
        dResult = 0.0f;
        result = 0.0f;
    }
     
    // TODO: 按 mode 实现位置式或增量式 PID，并限制积分和最终输出。
    // 完成前始终保持零输出。
   // pResult = iResult = dResult = result = 0.0f;
}

void PID::Clear()
{
    ref = fdb = 0.0f;
    err[0] = err[1] = err[2] = 0.0f;
    pResult = iResult = dResult = result = 0.0f;
}
