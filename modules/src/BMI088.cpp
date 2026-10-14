#include "BMI088.hpp"
#include "tx_api.h"
#include "bsp_pwm.hpp"
#include <cmath>

namespace BMI088
{
    /**
     * @brief BMI088 acc gyro 标定
     * @note 标定后的陀螺仪零偏存储在 Gyro_offset 中。
     * @attention 不管工作模式是blocking还是IT,标定时都是blocking模式,所以不用担心中断关闭后无法标定(RobotInit关闭了全局中断)
     * @attention 标定精度和等待时间有关。
     * @todo 将标定次数(等待时间)变为参数供设定
     * @section 整体流程为1.累加加速度数据计算gNrom()
     *                   2.累加陀螺仪数据计算零飘
     *                   3. 如果标定过程运动幅度过大,重新标定
     *                   4.保存标定参数
     */
    void cBMI088::Calibrate()
    {
        const int calib_samples = 4000; // 采样次数
        float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
        gyro_data_t temp_gyro;

        // 1. 清除旧的 Offset，防止叠加
        Gyro_offset[0] = 0.0f;
        Gyro_offset[1] = 0.0f;
        Gyro_offset[2] = 0.0f;

        // 2. 循环采样
        for (int i = 0; i < calib_samples; i++)
        {
            ReadGyroData(&temp_gyro); // 这里读取的是原始值（因为Offset已清零）
            gyro_sum[0] += temp_gyro.x;
            gyro_sum[1] += temp_gyro.y;
            gyro_sum[2] += temp_gyro.z;
            
            tx_thread_sleep(1); // 等待一个 ThreadX tick；实际耗时由 tick 频率决定。
        }

        // 3. 计算平均值作为零偏
        Gyro_offset[0] = gyro_sum[0] / calib_samples;
        Gyro_offset[1] = gyro_sum[1] / calib_samples;
        Gyro_offset[2] = gyro_sum[2] / calib_samples;
        
        // 如果零偏过大（例如超过 0.1 rad/s），可能是在运动中标定的，应报错或丢弃
        if (fabs(Gyro_offset[0]) > 0.1f || fabs(Gyro_offset[1]) > 0.1f || fabs(Gyro_offset[2]) > 0.1f)
        {
            self_test.CALIBRATE_ERR = true;
            // 恢复默认值或保留上次值
            Gyro_offset[0] = BMI088_GYRO_PRE_CALI_OFFSET_X; 
            Gyro_offset[1] = BMI088_GYRO_PRE_CALI_OFFSET_Y;
            Gyro_offset[2] = BMI088_GYRO_PRE_CALI_OFFSET_Z;
        }
        else
        {
            self_test.CALIBRATE_ERR = false;
        }
    }

    void cBMI088::TemperatureControl(float target_temp)
    {
        float temp_now = 0.0f;
        ReadAccTemperature(&temp_now);
        if (!(temp_now > 0.0f && temp_now < 100.0f))
         {
            PWM_SetDutyRatio(&HEATING_RESISTANCE_TIM, 0.0f, HEATING_RESISTANCE_CHANNEL);
            return;
         }
          TempPid.ref = target_temp;                     // IMU.hpp:66 的 TempPid
        TempPid.fdb = temp_now;
        TempPid.UpdateResult();
         float duty = TempPid.result;
        if (duty < 0.0f) duty = 0.0f;                  // 超过目标 → 不加热（不是反向制冷）
         if (duty > 1.0f) duty = 1.0f;
        PWM_SetDutyRatio(&HEATING_RESISTANCE_TIM, duty, HEATING_RESISTANCE_CHANNEL);
        // TODO: 温度读取完成后，用 TempPid 计算加热占空比并限制到 [0, 1]。
        // 未完成前保持加热关闭。
        (void)target_temp;
        PWM_SetDutyRatio(&HEATING_RESISTANCE_TIM, 0.0f, HEATING_RESISTANCE_CHANNEL);
    }

    void cBMI088::VerifyAccChipID()
    {
        uint8_t pRxData[2]; //< 读取两个字节，第一个是需要丢弃的无效字节，第二个是芯片 ID

        ReadReg(BMI088_CS_ACC, ACC_CHIP_ID_ADDR, pRxData, 2); //< 读取加速度计chip id
        tx_thread_sleep(1);
        //< 如果chip id不等于预设值,则加速度计ID错误,初始化错误
        if (pRxData[0] != ACC_CHIP_ID_VAL)
        {
            self_test.ACC_CHIP_ID_ERR = true;
            self_test.INIT_ERR = true;
        }
        else 
        {
            self_test.ACC_CHIP_ID_ERR = false;
        }

        
    }

    void cBMI088::VerifyGyroChipID()
    {
        uint8_t pRxData[2];                                                 //< 读取一个字节,chip id
        ReadReg(BMI088_CS_GYRO, GYRO_CHIP_ID_ADDR, pRxData, 2); //< 读取陀螺仪chip id
        tx_thread_sleep(1);
        //< 如果chip id不等于预设值,则陀螺仪ID错误,初始化错误
        if (pRxData[0] != GYRO_CHIP_ID_VAL)
        {
            self_test.GYRO_CHIP_ID_ERR = true;
            self_test.INIT_ERR = true;
        }
        else
        {
            self_test.GYRO_CHIP_ID_ERR = false;
        }

        tx_thread_sleep(1);
    }

    void cBMI088::VerifyAccData()
    {
        uint8_t erroeReg=0;
        uint8_t dataStatus=0;
        if(ReadReg(BMI088_CS_ACC, ACC_ERR_REG_ADDR, &erroeReg, 1) != HAL_OK||ReadReg(BMI088_CS_ACC, ACC_STATUS_ADDR, &dataStatus, 1) != HAL_OK)
        {
            self_test.ACC_DATA_ERR = true;
            return;
        }
        if((erroeReg&0x1D)!=0)
        {
            self_test.ACC_DATA_ERR = true;
            return;
        }
        if((dataStatus &0x80)==0)
        {
            self_test.ACC_DATA_ERR=true;
        }   
        if(self_test.ACC_DATA_ERR||!std::isfinite(acc_data.x)||!std::isfinite(acc_data.y)||!std::isfinite(acc_data.z))     
        {
            self_test.ACC_DATA_ERR=true;
            return;
        }
        self_test.ACC_DATA_ERR=false;
        // TODO: 检查加速度数据是否有效，并更新 ACC_DATA_ERR。
    }

    void cBMI088::VerifyGyroData()
    {
        uint8_t dataStatus=0;
        if(ReadReg(BMI088_CS_GYRO, GYRO_INT_STAT_1_ADDR, &dataStatus, 1) != HAL_OK)
        {
            self_test.GYRO_DATA_ERR = true;
            return;
        }
       
        if((dataStatus&GYRO_DRDY_ON)==0)
        {
            self_test.GYRO_DATA_ERR=true;
        }   
       ReadGyroData(&gyro_data);
       if(self_test.GYRO_DATA_ERR||!std::isfinite(gyro_data.x)||!std::isfinite(gyro_data.y)||!std::isfinite(gyro_data.z))     
        {
            self_test.GYRO_DATA_ERR=true;
            return;
        }
        self_test.GYRO_DATA_ERR=false;
        // TODO: 检查角速度数据是否有效，并更新 GYRO_DATA_ERR。
    }

    static bool GetPin(BMI088_SENSOR cs, GPIO_TypeDef **port, uint16_t *pin)
    {
        if(port==nullptr||pin==nullptr)
        {
            return false;
        }
        switch(cs)
        {
            case BMI088_CS_ACC:
            {
                *port=BMI088_ACC_PORT;
                *pin=BMI088_ACC_CS_PIN;
                return true;
            }
            case BMI088_CS_GYRO:
            {
                *port=BMI088_GYRO_PORT;
                *pin=BMI088_GYRO_CS_PIN;
                return true;
            }
            default:
                return false;
        }

    }

    void cBMI088::WriteReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
    {
        if(data == nullptr || len == 0)
        {
            self_test.INIT_ERR = true; 
            return;
        }
       GPIO_TypeDef *csPort=nullptr;

         uint16_t csPin=0;
         if(!GetPin(cs,&csPort,&csPin))
         {
            self_test.INIT_ERR = true; 
            return;
         }
         uint8_t tx[256]={};
         tx[0]=addr&BMI088_SPI_WRITE_CODE;
         memcpy(&tx[1],data,len);
         HAL_GPIO_WritePin(csPort,csPin,GPIO_PIN_RESET);
         HAL_StatusTypeDef status=HAL_SPI_Transmit(&BMI088_SPI_HANDLE,tx,static_cast<uint16_t>(len+1),100);
         HAL_GPIO_WritePin(csPort,csPin,GPIO_PIN_SET);
         if(status!=HAL_OK)
         {
            self_test.INIT_ERR = true; 
            
         }
        // TODO: 根据 cs 拉低对应 GPIO 片选；按 SPI 写协议发送地址和数据，
        // 最后释放片选，并处理通信失败与必要的延时。
        (void)cs;
        (void)addr;
        (void)data;
        (void)len;
    }

   HAL_StatusTypeDef cBMI088::ReadReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
    {
        

         if (data == nullptr || len == 0)
         {
             self_test.INIT_ERR = true;
             return HAL_ERROR;
          }

         GPIO_TypeDef *csPort = nullptr;
            uint16_t csPin = 0;

         if (!GetPin(cs, &csPort, &csPin))
          {
             self_test.INIT_ERR = true;
             memset(data, 0, len);
             return HAL_ERROR;
        }
        const uint16_t extraBytes =
        (cs == BMI088_CS_ACC) ? 2U : 1U;

        const uint16_t transferSize =
        static_cast<uint16_t>(len) + extraBytes;//

        uint8_t tx[257] = {};
        uint8_t rx[257] = {};

        uint8_t tx1 = addr | BMI088_SPI_READ_CODE;
    
        tx[0] = addr | BMI088_SPI_READ_CODE;

    
        for (uint16_t i = 1; i < transferSize; ++i)
        {
             tx[i] = 0xFF;
        }

        HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);

        // HAL_StatusTypeDef status =HAL_SPI_TransmitReceive(&BMI088_SPI_HANDLE, tx, rx,transferSize, 100);
        HAL_StatusTypeDef status =
         HAL_SPI_Transmit(&BMI088_SPI_HANDLE, &tx1, 1, 100);

        if (status == HAL_OK)
        {
            status = HAL_SPI_Receive(&BMI088_SPI_HANDLE, rx, transferSize, 100);
        }


         HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

        if (status != HAL_OK)
         {
            memset(data, 0, len);
            self_test.INIT_ERR = true;

             if (cs == BMI088_CS_ACC)
            {
                self_test.ACC_DATA_ERR = true;
             }
             else
            {
                 self_test.GYRO_DATA_ERR = true;
            }

        return status;
        }

    
        const uint16_t dataOffset =(cs == BMI088_CS_ACC) ? 2U : 1U;

        memcpy(data, &rx[dataOffset], len);
        // TODO: 根据 cs 拉低对应 GPIO 片选；按 SPI 读协议发送地址，//这里将函数返回值改成了HAL_StatusTypeDef
        // 丢弃加速度计返回的首个无效字节，读取 len 字节并释放片选。
        // 未实现前清零缓冲区，避免芯片 ID 校验读取未初始化数据。
       return HAL_OK;
        
    }

    void cBMI088::Config()
    {
        tx_thread_sleep(10); //< 等待系统稳定

       

        // 加速度计上电默认 I2C 模式：先盲读一次切到 SPI，读到的值无效，必须丢弃
        uint8_t dummy = 0;
        ReadReg(BMI088_CS_ACC, ACC_CHIP_ID_ADDR, &dummy, 1);
        tx_thread_sleep(1);
        // 陀螺仪首次读也常返回无效值，一起丢掉，代价一次 SPI 传输
        ReadReg(BMI088_CS_GYRO, GYRO_CHIP_ID_ADDR, &dummy, 1);
        tx_thread_sleep(1);

    // ↓ 下面保持原来的软复位 / 上电 / 量程流程不动

        /*-------------------------------------加速度计初始化-------------------------------------*/
        
        //< 先软重启，清空所有寄存器
        uint8_t pTxData;
        pTxData = ACC_SOFTRESET_VAL;
        WriteReg(BMI088_CS_ACC, ACC_SOFTRESET_ADDR, &pTxData, 1);
        tx_thread_sleep(100); //< 延时100ms,重启需要时间

        //< 打开加速度计电源
        pTxData = ACC_PWR_CTRL_ON;
        WriteReg(BMI088_CS_ACC, ACC_PWR_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(150);

        //< 加速度计变成正常模式
        pTxData = ACC_PWR_CONF_ACT;
        WriteReg(BMI088_CS_ACC, ACC_PWR_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(10); //

        //< 测量范围
        pTxData = ACC_RANGE_6G;
        WriteReg(BMI088_CS_ACC, ACC_RANGE_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0xAB;
        WriteReg(BMI088_CS_ACC, ACC_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x08;
        WriteReg(BMI088_CS_ACC, INT1_IO_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x04;
        WriteReg(BMI088_CS_ACC, INT_MAP_DATA_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        /*-------------------------------------陀螺仪初始化-------------------------------------*/
        //< 先软重启，清空所有寄存器
        pTxData = GYRO_SOFTRESET_VAL;
        WriteReg(BMI088_CS_GYRO, GYRO_SOFTRESET_ADDR, &pTxData, 1);
        tx_thread_sleep(100); //< 延时100ms,重启需要时间

        pTxData = GYRO_RANGE_2000_DEG_S;
        WriteReg(BMI088_CS_GYRO, GYRO_RANGE_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_ODR_2000Hz_BANDWIDTH_230Hz | GYRO_LPM1_SUS;
        WriteReg(BMI088_CS_GYRO, GYRO_BANDWIDTH_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_LPM1_NOR;
        WriteReg(BMI088_CS_GYRO, GYRO_LPM1_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_DRDY_ON;
        WriteReg(BMI088_CS_GYRO, GYRO_INT_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x00;
        WriteReg(BMI088_CS_GYRO, GYRO_INT3_INT4_IO_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x01;
        WriteReg(BMI088_CS_GYRO, GYRO_INT3_INT4_IO_MAP_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms
    }


    void cBMI088::ReadAccData(acc_data_t *data)
    {
        if(data==nullptr)
        {
            return;
        }
        uint8_t bytes[ACC_XYZ_LEN]={};
        if(ReadReg(BMI088_CS_ACC,ACC_X_LSB_ADDR,bytes,ACC_XYZ_LEN)!=HAL_OK)//ReadReg已处理过Dummy
        {
            data->x=data->y=data->z=0.0f;
            self_test.ACC_DATA_ERR=true;
            return;
        }
        const int16_t rawX=static_cast<int16_t>((bytes[1]<<8)|bytes[0]);
        const int16_t rawY=static_cast<int16_t>((bytes[3]<<8)|bytes[2]);
        const int16_t rawZ=static_cast<int16_t>((bytes[5]<<8)|bytes[4]);
        data->x=rawX*IMU_ACCEL_6G_SEN;
        data->y=rawY*IMU_ACCEL_6G_SEN;
        data->z=rawZ*IMU_ACCEL_6G_SEN;
        self_test.ACC_DATA_ERR=false;
        // TODO: 按 BMI088 加速度计 SPI 协议丢弃首个无效字节，拼接三轴有符号原始值，
        // 再按配置的量程换算为 m/s²，写入 data。
       
    }

    void cBMI088::ReadGyroData(gyro_data_t *data)
    {
        if (data == nullptr)
        {
            return;
        }

         uint8_t bytes[GYRO_XYZ_LEN] = {};

        if (ReadReg(BMI088_CS_GYRO, GYRO_RATE_X_LSB_ADDR,bytes,GYRO_XYZ_LEN) != HAL_OK)
         {
            data->x = data->y = data->z = 0.0f;
            self_test.GYRO_DATA_ERR = true;
            return;
         }

         const int16_t rawX = static_cast<int16_t>((static_cast<uint16_t>(bytes[1]) << 8) | bytes[0]);
        const int16_t rawY = static_cast<int16_t>((static_cast<uint16_t>(bytes[3]) << 8) | bytes[2]);
        const int16_t rawZ = static_cast<int16_t>( (static_cast<uint16_t>(bytes[5]) << 8) | bytes[4]);

         data->x = rawX * IMU_GYRO_2000_SEN - Gyro_offset[0];
        data->y = rawY * IMU_GYRO_2000_SEN - Gyro_offset[1];
        data->z = rawZ * IMU_GYRO_2000_SEN - Gyro_offset[2];

        self_test.GYRO_DATA_ERR = false;
        // TODO: 读取并拼接陀螺仪三轴原始值，按配置量程换算为 rad/s，
        // 减去 Gyro_offset 后写入 data。
        
    }

    void cBMI088::ReadAccTemperature(float *temp)
    {
        if(temp==nullptr)
        {
            return;
        }
        uint8_t bytes[TEMP_LEN]={};
        if(ReadReg(BMI088_CS_ACC,TEMP_MSB_ADDR,bytes,TEMP_LEN)!=HAL_OK)//ReadReg已处理过Dummy
        {
            *temp=0.0f;
            self_test.TEMP_CTRL_ERR=true;
            return;
        }
        uint16_t raw_Temp=(static_cast<uint16_t>(bytes[0])<<3)|(bytes[1]>>5);
        if((raw_Temp&0x0400)!=0)          //是负的
        {
            raw_Temp-=2048;
        }
        *temp=raw_Temp*TEMP_UNIT+TEMP_BIAS;
        // TODO: 丢弃加速度计读取时的首个无效字节，解析 11 位有符号温度并换算为摄氏度。
        
    }

}

