#include "M2006.hpp"

M2006::M2006()
{
    gearBox = GearBox_M2006;
    maxCurrent = 1000;                 //先给1A跑通再改成3A
    speedPid.Tuning(10.0f, 0.0f, 0.0f); 
    speedPid.maxOut= static_cast<float>(maxCurrent); 
    speedPid.maxIOut= static_cast<float>(maxCurrent); 
    speedPid.Clear();

    positionPid.Tuning(10.0f, 0.0f, 0.0f);
    positionPid.maxOut= 40.0f;           //转速比额定小一点
    positionPid.maxIOut= 0.0f;
    positionPid.Clear();

    // TODO: 根据 M2006 手册设置减速比、允许电流和速度/位置环 PID 参数。
    // 基类初始化为松开模式、零电流；完成配置前不允许驱动电机。
}

void M2006::setOutput()
{
    if(MotorState == MOTOR_OFFLINE)
    {
        currentSet = 0;
        return;
    }
    float out=0.0f;
    switch(controlMode)
    {
        case RELAX_MODE:
            out = 0.0f;
            break;
        case SPD_MODE:
            speedPid.ref = speedSet;
            speedPid.fdb = motorFeedback.speedFdb;
            speedPid.UpdateResult();
            out = speedPid.result;
            break;
        case POS_MODE:
            positionPid.ref = positionSet;
            positionPid.fdb = motorFeedback.positionFdb;
            positionPid.UpdateResult();
            speedPid.ref = positionPid.result;
            speedPid.fdb = motorFeedback.speedFdb;
            speedPid.UpdateResult();
            out = speedPid.result;
            break;
        default:
            out = 0.0f;
            break;
    }
    currentSet=static_cast<int16_t>(Numeric::LimitABS(out,static_cast<float>(maxCurrent)));
    // TODO: 分别实现松开、速度和位置模式；位置模式需要串联位置环与速度环。
    // TODO: 将 PID 输出限制在 maxCurrent 范围内后写入 currentSet。

}

M2006::MotorStateTypedef M2006::AliveCheck()
{
    if(AliveFlag != Pre_AliveFlag)
    {
        MotorState = MOTOR_ONLINE;
        Pre_AliveFlag = AliveFlag;
        return MotorState;
    }
    else
    {
        MotorState = MOTOR_OFFLINE;
        return MotorState;
    }
    
    // TODO: 比较 AliveFlag 与 Pre_AliveFlag，更新在线状态并保存本次计数。
    MotorState = MOTOR_OFFLINE;
    return MotorState;
}
