#include "bsp_can.hpp"
#include "DJIMotorHandler.hpp"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
 * @brief 初始化CAN滤波器配置。
 * 设置CAN硬件的滤波器，用于优化接收数据的处理。
 * 更多信息，请参考原文，链接：https://blog.csdn.net/weixin_54448108/article/details/128570593
 */
void CAN_Init(void)
{
    CAN_FilterTypeDef can_filter_st;                   ///< 定义过滤器结构体
    can_filter_st.FilterActivation = ENABLE;           ///< ENABLE使能过滤器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;  ///< 设置过滤器模式--标识符屏蔽位模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT; ///< 过滤器的位宽 32 位
    can_filter_st.FilterIdHigh = 0x0000;               ///< ID高位
    can_filter_st.FilterIdLow = 0x0000;                ///< ID低位
    can_filter_st.FilterMaskIdHigh = 0x0000;           ///< 过滤器掩码高位
    can_filter_st.FilterMaskIdLow = 0x0000;            ///< 过滤器掩码低位

    can_filter_st.FilterBank = 0;                                      ///< 过滤器组-双CAN可指定0~27
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;                 ///< 与过滤器组管理的 FIFO
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);                      ///< HAL库配置过滤器函数
    HAL_CAN_Start(&hcan1);                                             ///< 使能CAN1控制器
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); ///< 使能CAN的各种中断

    can_filter_st.SlaveStartFilterBank = 14;                           ///< 双CAN模式下规定CAN的主从模式的过滤器分配，从过滤器为14
    can_filter_st.FilterBank = 14;                                     ///< 过滤器组-双CAN可指定0~27
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);                      ///< HAL库配置过滤器函数
    HAL_CAN_Start(&hcan2);                                             ///< 使能CAN2控制器
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); ///< 使能CAN的各种中断
}

void CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t Id, uint8_t *msg, uint16_t len)
{
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
    {
        // 如果邮箱已满，可以选择等待或者采取其他操作
        // 例如，可以添加一个超时机制或者直接返回错误状态
        // Monitor::instance()->Log_Messages(Monitor::WARNING, (uint8_t *)"CAN Tx Mailbox is full\r\n");
        return;
    }
    {
    // 检查 CAN 控制器句柄
    if (hcan == nullptr)
    {
        return;
    }

    // 本函数只发送标准帧：
    // 标准帧 ID 是 11 位，因此范围为 0x000～0x7FF
    if (Id > 0x7FFU)
    {
        return;
    }

    // CAN 数据区最多 8 字节
    if (len > 8U)
    {
        return;
    }

    // 有数据要发送时，数据指针不能是空指针
    if ((len > 0U) && (msg == nullptr))
    {
        return;
    }

    // 检查发送邮箱是否至少有一个空位
    // 邮箱是 CAN 控制器暂存待发送报文的位置
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        return;
    }

    CAN_TxHeaderTypeDef txHeader = {};

    txHeader.StdId = Id;                 // 标准帧标识符
    txHeader.ExtId = 0U;                 // 这里不使用扩展帧标识符
    txHeader.IDE = CAN_ID_STD;           // 使用标准帧
    txHeader.RTR = CAN_RTR_DATA;         // 发送数据帧，不是远程请求帧
    txHeader.DLC = len;                  // 实际数据长度，0～8 字节
    txHeader.TransmitGlobalTime = DISABLE;

    uint32_t mailbox = 0U;//vv

    // 将报文交给 HAL 和 CAN 控制器发送
    if (HAL_CAN_AddTxMessage(hcan, &txHeader, msg, &mailbox) != HAL_OK)
    {
        // 发送请求未能提交时返回。
        // 调试时可在这里加日志或错误计数。
        return;
    }
 }// TODO: 校验标准帧 ID 和 DLC (0~8)，构造帧头并调用 HAL_CAN_AddTxMessage。
    (void)Id;
    (void)msg;
    (void)len;
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader = {};
    uint8_t rxData[8] = {};

    // 从 FIFO0 取出刚收到的报文
    if (HAL_CAN_GetRxMessage(
            hcan,
            CAN_RX_FIFO0,
            &rxHeader,
            rxData) != HAL_OK)
    {
        return;
    }

    // 只处理标准数据帧，且长度必须是 8 字节
    if (rxHeader.IDE != CAN_ID_STD ||
        rxHeader.RTR != CAN_RTR_DATA ||
        rxHeader.DLC != 8U)
    {
        return;
    }

    // 项目约定：电机反馈 ID 是 0x201～0x208
    if (rxHeader.StdId < 0x201U || rxHeader.StdId > 0x208U)
    {
        return;
    }

    // 把 0x201～0x208 转为数组索引 0～7
    const int motorIndex = static_cast<int>(rxHeader.StdId - 0x201U);

    // 交给电机管理器，根据 CAN 总线和索引更新对应电机
    DJIMotorHandler::Instance()->updateFeedback(
        hcan,
        rxData,
        motorIndex);
}




// TODO: 实现 CAN 接收回调，根据总线和标准帧 ID 分发电机反馈。
