#include"threads.hpp"
#include"controlthread.hpp"
#include"DJIMotorHandler.hpp"
#include"M2006.hpp"
#include"bsp.hpp"
#include"can.h"
#include<cmath>
namespace
{
    //参数
    constexpr uint16_t kCanId       = 0x201;   // 电机拨码 ID
    constexpr float    kPosTarget   = 3.14f;   // [rad] 位置目标（输出轴累计角）
    constexpr float    kSpeedTarget = 30.0f;   // [rad/s] 速度目标（纯速度环测试用），     
    constexpr float    kTempLimit   = 70.0f;   // [℃] 过温保护                           根据实际热性能调整
    constexpr float    kSpeedLimit  = 400.0f;  // [rad/s] 数值合理性上限                  可能要微调
    constexpr uint32_t kArmTicks    = 100;     // 连续在线 100 ms 才允许输出               看can的通信频率
    constexpr uint32_t kLostTicks   = 50;      // 连续 50 ms 无新帧 → 停机                 

    DJIMotor::MotorControlModeType g_mode     = DJIMotor::POS_MODE; // 双环；纯速度环改 SPD_MODE
    volatile float g_speedSet = kSpeedTarget;                       //volatile每次查内存
    volatile float g_posSet   = kPosTarget;
    volatile bool  g_enable   = true;
    volatile bool  g_fault    = false;
    M2006 g_motor;
    DJIMotor *const g_motors[]={&g_motor};                       //便于拓展

}

void MotorThread_Entry(ULONG thread_input)
{
    (void)thread_input;
    bsp_Init();
    DJIMotorHandler::Instance()->registerMotor(&g_motor,&hcan1,kCanId);

    g_motor.controlMode=g_mode;
    g_motor.speedSet=g_speedSet;
    g_motor.positionSet=g_posSet;
    g_motor.speedPid.Clear();
    g_motor.positionPid.Clear();

    uint32_t lastAlive=g_motor.AliveFlag;;
    uint32_t lostCnt=0;
    uint32_t onlineCut=0;
    bool armed=false;

    for(;;)
    {
        DJIMotorHandler::Instance()->AllMotorAliveCheck();

        if(g_motor.AliveFlag!=lastAlive)
        {
            lastAlive=g_motor.AliveFlag;
            lostCnt=0;
        }
        else if(lostCnt<kLostTicks) 
        {
            lostCnt++;
        }

        const DJIMotor::MotorFeedBack &fb=g_motor.motorFeedback;
        const bool dataOk=std::isfinite(fb.speedFdb)&&std::isfinite(fb.positionFdb)&&std::fabs(fb.speedFdb)<kSpeedLimit&&fb.temperatureFdb<kTempLimit;
        const bool linkOk=(g_motor.MotorState==DJIMotor::MOTOR_ONLINE)&&(lostCnt==0);

        if(dataOk&&linkOk)
        {
            if(onlineCut<kArmTicks) onlineCut++;
        }
        else
        {  
            onlineCut=0;

        }

        armed=(onlineCut>=kArmTicks&&g_enable);
        g_fault=!(linkOk&&dataOk);

        if(armed)
        {
            g_motor.controlMode=g_mode;
            g_motor.speedSet=g_speedSet;
            g_motor.positionSet=g_posSet;
        }
        else
        {
            if(g_motor.controlMode!= DJIMotor::RELAX_MODE)
            {
                g_motor.controlMode=DJIMotor::RELAX_MODE;
                g_motor.speedPid.Clear();                     //清积分，防止恢复时输出突跳（）
                g_motor.positionPid.Clear();
            }
            g_motor.speedSet=0.0f;
        }

        for(DJIMotor *m:g_motors)
        {
            m->setOutput();
        }

        DJIMotorHandler::Instance()->sendControlData();

        tx_semaphore_put(&sem_motor);                        
        tx_thread_sleep(1);
    }

}

namespace ControlThread
{
    void SetSpeedTarget(float v)    { g_speedSet = v; }
    void SetPositionTarget(float v) { g_posSet   = v; }
    void SetEnable(bool on)         { g_enable   = on; }
    bool IsFault()                  { return g_fault; }
}
