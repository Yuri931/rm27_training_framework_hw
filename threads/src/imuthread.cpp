#include "threads.hpp"
#include "imuthread.hpp"
#include "LED.hpp"
#include "tx_api.h"

BMI088::cBMI088 imu;

/* ---- 临时诊断:回读配置寄存器,用完删掉 ---- */
/* 放全局是为了在调试器里任何地方都能 Watch。下标含义:
 *  [0] ACC  0x7C PWR_CONF   期望 0x00(0x03 = 还挂起)
 *  [1] ACC  0x7D PWR_CTRL   期望 0x04(0x00 = 没上电)
 *  [2] ACC  0x40 ACC_CONF   期望 0xAB
 *  [3] ACC  0x41 ACC_RANGE  期望 0x01
 *  [4] ACC  0x03 ACC_STATUS 看 bit7(drdy)
 *  [5] ACC  0x00 CHIP_ID    期望 0x1E
 *  [6] GYRO 0x11 LPM1       期望 0x00(0x80 = 还挂起)
 *  [7] GYRO 0x0F RANGE      期望 0x00
 *  [8] GYRO 0x10 BANDWIDTH  期望 0x81
 *  [9] GYRO 0x00 CHIP_ID    期望 0x0F
 */
uint8_t cfg_readback[10] = {0};
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
    
  //  uint8_t dbg_acc1[8], dbg_acc2[8], dbg_gyro1[8], dbg_gyro2[8];   // 全局，方便 Watch


//     tx_thread_sleep(100);
// RawRead(false, 0x00, dbg_gyro_pre, 8);    // Config 之前：陀螺还活着吗？
// RawRead(true,  0x00, dbg_acc_pre,  8);
// imu.Config();
// RawRead(false, 0x00, dbg_gyro_post, 8);   // Config 之后


// tx_thread_sleep(100);
// // // 连读两次，对比"第一次"和"第二次"：
// RawRead(true,  0x00, dbg_acc1,  8);
// RawRead(false, 0x00, dbg_gyro1, 8);
// imu.Config();

//     /* ---- 临时诊断:Config 之后立刻回读,看配置到底写进去没有 ---- */
//     imu.ReadReg(BMI088::BMI088_CS_ACC,  0x7C, &cfg_readback[0], 1);
//     imu.ReadReg(BMI088::BMI088_CS_ACC,  0x7D, &cfg_readback[1], 1);
//     imu.ReadReg(BMI088::BMI088_CS_ACC,  0x40, &cfg_readback[2], 1);
//     imu.ReadReg(BMI088::BMI088_CS_ACC,  0x41, &cfg_readback[3], 1);
//     imu.ReadReg(BMI088::BMI088_CS_ACC,  0x03, &cfg_readback[4], 1);
//     imu.ReadReg(BMI088::BMI088_CS_ACC,  0x00, &cfg_readback[5], 1);
//     imu.ReadReg(BMI088::BMI088_CS_GYRO, 0x11, &cfg_readback[6], 1);
//     imu.ReadReg(BMI088::BMI088_CS_GYRO, 0x0F, &cfg_readback[7], 1);
//     imu.ReadReg(BMI088::BMI088_CS_GYRO, 0x10, &cfg_readback[8], 1);
//     imu.ReadReg(BMI088::BMI088_CS_GYRO, 0x00, &cfg_readback[9], 1);
//     /* ---- 诊断结束 ---- */

// RawRead(true,  0x00, dbg_acc2,  8);


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

