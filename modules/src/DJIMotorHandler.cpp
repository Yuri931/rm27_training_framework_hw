#include "DJIMotorHandler.hpp"
#include"bsp_can.hpp"

void DJIMotorHandler::registerMotor(DJIMotor *motor, CAN_HandleTypeDef *hcan, uint16_t canId)
{
    if (motor == nullptr || hcan == nullptr) return;
    if (canId < 0x201U || canId > 0x208U)    return;

    const int bus = (hcan == &hcan1) ? 0 : (hcan == &hcan2) ? 1 : -1;
    if (bus < 0) return;
    const int index = static_cast<int>(canId - 0x201U);      // 0~7

    DJIMotorList[bus][index] = motor;
    motor->hcan  = hcan;
    motor->canId = canId;

    if (bus == 0) 
    { 
        if (index < 4) 
        {
            CAN1_0x200_Exist = true;
         }
         else
         {
            CAN1_0x1FF_Exist = true;
         } 
    }
    else          
    {
         if (index < 4) 
         {
            CAN2_0x200_Exist = true;
          }
        else 
       {
            CAN2_0x1FF_Exist = true;
         }
    // TODO: 校验 CAN 句柄和 0x201~0x208 ID，再登记到对应总线与槽位。
    // TODO: 按 ID 设置对应的 0x200/0x1FF 报文存在标志。
   
}
}

namespace {
inline void PutCurrent(uint8_t *buf, int slot, int16_t cur)
{
    buf[slot * 2]     = static_cast<uint8_t>((cur >> 8) & 0xFF);   // 大端：高字节在前
    buf[slot * 2 + 1] = static_cast<uint8_t>(cur & 0xFF);
}
}

void DJIMotorHandler::sendControlData()
{

    if (CAN1_0x200_Exist)
    {
        for (int i = 0; i < 4; i++)
            PutCurrent(can1_send_data_0, i, DJIMotorList[0][i] ? DJIMotorList[0][i]->currentSet : 0);
        CAN_Transmit(&hcan1, 0x200U, can1_send_data_0, 8);
    }
    if (CAN1_0x1FF_Exist)  {
         for (int i = 0; i < 4; i++)
            PutCurrent(can1_send_data_1, i, DJIMotorList[0][i] ? DJIMotorList[0][i]->currentSet : 0);
        CAN_Transmit(&hcan1, 0x1FFU, can1_send_data_1, 8); /* 槽位 4~7 → can1_send_data_1，ID 0x1FF */
     }
    if (CAN2_0x200_Exist)  { 
         for (int i = 0; i < 4; i++)
            PutCurrent(can2_send_data_0, i, DJIMotorList[0][i] ? DJIMotorList[0][i]->currentSet : 0);
        CAN_Transmit(&hcan2, 0x200U, can2_send_data_0, 8);
        /* can2_send_data_0，&hcan2，ID 0x200 */ 
    }
    if (CAN2_0x1FF_Exist)  { 
         for (int i = 0; i < 4; i++)
            PutCurrent(can2_send_data_1, i, DJIMotorList[0][i] ? DJIMotorList[0][i]->currentSet : 0);
        CAN_Transmit(&hcan2, 0x21FFU, can2_send_data_1, 8);
        /* can2_send_data_1，&hcan2，ID 0x1FF */ 
    }
    // TODO: 将各电机 currentSet 按大端序放入正确的 8 字节控制帧。
    // TODO: 仅发送包含已注册电机的报文；未完成前不发送 CAN 电流命令。
}

void DJIMotorHandler::updateFeedback(CAN_HandleTypeDef *hcan, uint8_t *rx_data, int index)
{
     if (rx_data == nullptr || index < 0 || index > 7) return;
    const int bus = (hcan == &hcan1) ? 0 : (hcan == &hcan2) ? 1 : -1;
    if (bus < 0) return;
    DJIMotor *motor = DJIMotorList[bus][index];
    if (motor == nullptr) return;                 // 该 ID 未注册，直接丢弃
    UpdateSensorData(motor, rx_data);
    // TODO: 校验总线、反馈数据和索引，再找到对应电机并调用 UpdateSensorData。
    (void)hcan;
    (void)rx_data;
    (void)index;
}

void DJIMotorHandler::UpdateSensorData(DJIMotor *motor, uint8_t *can_data)
{
    const uint8_t *d=can_data;
     const uint16_t ecd  = static_cast<uint16_t>((d[0] << 8) | d[1]);   // 0~8191
    const int16_t  rpm  = static_cast<int16_t>((d[2] << 8) | d[3]);    // 转子 rpm
    const int16_t  raw  = static_cast<int16_t>((d[4] << 8) | d[5]);    // 电流 raw
    const uint8_t  temp = d[6];

    const bool first = (motor->AliveFlag == 0);            // 首帧不做差分，否则位置突跳一整圈
    int16_t delta = first ? 0 : static_cast<int16_t>(ecd - motor->motorFeedback.last_ecd);
    if (delta >  4096) delta -= 8192;                      // 编码器回绕
    if (delta < -4096) delta += 8192;

    const float ratio = (motor->gearBox == GearBox_M2006) ? 36.0f : 1.0f;  // 减速比

    motor->motorFeedback.last_ecd        = static_cast<int16_t>(ecd);
    motor->motorFeedback.ecd             = ecd;
    motor->motorFeedback.speed_rpm       = rpm;
    motor->motorFeedback.currentFdb      = static_cast<float>(raw);
    motor->motorFeedback.lastSpeedFdb    = motor->motorFeedback.speedFdb;
    motor->motorFeedback.speedFdb        = static_cast<float>(rpm) * (2.0f * 3.14159265f / 60.0f) / ratio;
    motor->motorFeedback.lastPositionFdb = motor->motorFeedback.positionFdb;
    motor->motorFeedback.positionFdb    += static_cast<float>(delta) * (2.0f * 3.14159265f / 8192.0f) / ratio;
    motor->motorFeedback.temperatureFdb  = static_cast<float>(temp);

    motor->AliveFlag++;          //  AliveCheck（M2006.cpp:61）靠这个
    // TODO: 从 CAN 数据解析编码器、转速、电流和温度。
    // TODO: 处理编码器回绕，并按减速比换算输出轴位置和角速度。
    
}

void DJIMotorHandler::AllMotorAliveCheck()
{
    for (int bus = 0; bus < 2; bus++)
        for (int i = 0; i < 8; i++)
            if (DJIMotorList[bus][i] != nullptr) (void)DJIMotorList[bus][i]->AliveCheck();
    // TODO: 遍历已注册电机并调用 AliveCheck。
}
