#include "threads.hpp"
#include "imuthread.hpp"
#include "LED.hpp"
#include "tx_api.h"

BMI088::cBMI088 imu;

void ImuThread_Entry(ULONG thread_input)
{
    (void)thread_input;
     tx_thread_sleep(100);          // 上电稳定（手册要求上电后 ≥1ms 才可通信，这里给足 100ms）
    imu.Config();                 
    imu.VerifyAccChipID();         
    imu.VerifyGyroChipID();        
    imu.Calibrate();

    const bool init_ok= !imu.self_test.ACC_CHIP_ID_ERR &&!imu.self_test.GYRO_CHIP_ID_ERR &&!imu.self_test.CALIBRATE_ERR;
    imu.self_test.INIT_ERR=!init_ok;
    if(init_ok)
    {
        tx_semaphore_put(&sem_init_ok);
    }
    for(;;)
    {
        imu.ReadAccData(&imu.acc_data);                      // 三轴加速度 m/s²，写入 acc_data.x/y/z
        imu.ReadGyroData(&imu.gyro_data);                    // 三轴角速度 rad/s，已减 Gyro_offset
        imu.ReadAccTemperature(&imu.acc_data.temperature);   // 温度 ℃，写进 acc_data.temperature
        imu.VerifyAccData();                                 // 更新 ACC_DATA_ERR
        imu.VerifyGyroData();                                //  更新 GYRO_DATA_ERR
        imu.TemperatureControl(40.0f);                       //加热控制,目标40

        tx_semaphore_put(&sem_imu);
        tx_thread_sleep(1);
    }
}

