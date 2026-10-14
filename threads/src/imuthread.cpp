#include "threads.hpp"
#include "imuthread.hpp"
#include "LED.hpp"
#include "tx_api.h"

BMI088::cBMI088 imu;
static void RawRead(bool acc, uint8_t addr, uint8_t *rx, uint16_t n)
{
    GPIO_TypeDef *port = acc ? GPIOA : GPIOB;
    uint16_t      pin  = acc ? GPIO_PIN_4 : GPIO_PIN_0;   // C板：ACC=PA4，GYRO=PB0
    uint8_t tx[8] = {0};
    tx[0] = addr | 0x80;
    for (uint16_t i = 1; i < n; ++i) tx[i] = 0xFF;
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, n, 100);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

void ImuThread_Entry(ULONG thread_input)
{
    (void)thread_input;
     tx_thread_sleep(100);          // 上电稳定（手册要求上电后 ≥1ms 才可通信，这里给足 100ms）
    imu.Config();   
    
    uint8_t dbg_acc1[8], dbg_acc2[8], dbg_gyro1[8], dbg_gyro2[8];   // 全局，方便 Watch


// 连读两次，对比"第一次"和"第二次"：
RawRead(true,  0x00, dbg_acc1,  8);
RawRead(true,  0x00, dbg_acc2,  8);
RawRead(false, 0x00, dbg_gyro1, 8);
RawRead(false, 0x00, dbg_gyro2, 8);

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

