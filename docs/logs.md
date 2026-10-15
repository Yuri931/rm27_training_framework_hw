##0.灯效信息：
1. 颜色分辨"谁出问题"，频率分辨"严重程度"：正常 = 慢闪心跳，异常 = 快闪，致命 = 常亮。

初始化没过：蓝灯常亮
imu丢失，蓝灯快闪
电机掉线，红灯快闪
2. 三态语义：不亮 = 没跑/没上电；常亮 = 卡死或初始化没过；闪烁 = 运行中。
3. 上电自检跑马灯：三色依次亮一遍，确认 LED 本身正常（也顺便验证了 MX_GPIO_Init 和引脚配置），然后再进入正常指示。
4. 错误仲裁：多个错误同时存在时只显示优先级最高的那一个，不要三个灯一起乱闪——那样什么信息都读不出来。

 注意controlthread里面volatile bool  g_enable   = false;，要在调试器里手动 ControlThread::SetEnable(true)，或者先把目标改成 0。

 ##1.找不到BMI088.hpp文件
 最可能的原因：IDE 把 board/ 当成了工程根目录。工程里有两个 CMakeLists：

 解决：再烧录一次即可；


图1：链接错误

解决：stlink线松了。





 ##2.板子初始化不通过，一直亮蓝灯

imu.self_test.ACC_CHIP_ID_ERR为true，else语句写错；
 imu.self_test.GYRO_CHIP_ID_ERR为true，待查
 ID读错
 解决

 先把原始字节抓出来，一次定案

ReadReg 会剥字节，看不清真相。临时加个裸读函数抓原始字节（放 imuthread.cpp，在 ImuThread_Entry 里 imu.Config() 之后调）：

uint8_t dbg_acc1[4], dbg_acc2[4], dbg_gyro1[4], dbg_gyro2[4];   // 全局，方便 Watch

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
// 连读两次，对比"第一次"和"第二次"：
RawRead(true,  0x00, dbg_acc1,  4);
RawRead(true,  0x00, dbg_acc2,  4);
RawRead(false, 0x00, dbg_gyro1, 4);
RawRead(false, 0x00, dbg_gyro2, 4);


陀螺仪：全是 0xFF = MISO 一直是空闲高电平，没有任何芯片在驱动它
9
gyro1 = 00 FF FF FF
gyro2 = FF FF FF

注意这里连 0x80 都没有——所以也不是首读无效的问题（首读无效会返回 0x80 这种具体值）。这个是陀螺根本没应答：要么没被选中，要么它压根不在 SPI 模式。

先做这个实验：把 SPI 时钟降下来

（假的，用的是pclk，为84Hz不超）你的 SPI_BAUDRATEPRESCALER_16 = 168/16 = 10.5 MHz，超了 BMI088 手册的 10 MHz 上限。"响应稳定地晚一个字节"和"某个 die 完全不响应"都符合超频跑在边缘上的表现——这可以同时解释你这两个现象。

频率168/16>10超时了，改动：hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;、

##3.问题：线程进不了
原因：generate code覆盖了原来改过的宏，后内存池重新被改成1024；
解决：0.（无效）board/AZURE_RTOS/App/app_azure_rtos_config.h — 在 USER CODE BEGIN EC 用户区里覆盖宏（CubeMX 再生成也不会被冲掉）：
        #undef  TX_APP_MEM_POOL_SIZE
        #define TX_APP_MEM_POOL_SIZE    (16*1024)
2. threads/src/alivethread.cpp — 分配失败不再静默跳过，改成 Threads_InitFailed()：亮红+蓝灯并 __disable_irq() 停下，下次这种问题直接通过灯效显示。
3. 改成*8：board/AZURE_RTOS/App/app_azure_rtos_config.h：#defineTX_APP_MEM_POOL_SIZE  1024*8

##4alivethread取不了心跳
原因：init_ok因为&tx_semaphore_get(&sem_init_ok,TX_NO_WAIT)！=success不通过
取出13，表示TX_NO_INSTANCE，说明这个通知还没有发出。没有put过，因为imu初始化没通过(上面的问题没解决导致的)
核心原因就是：在 alivethread 执行到这一行时，负责 tx_semaphore_put(&sem_init_ok) 的那段代码还没有运行，或者运行失败了。

##4.电机不动，controlthread也进不了
    原因：忘记controlthread里面volatile bool  g_enable   = false;，要在调试器里手动 ControlThread::SetEnable(true)，或者先把目标改成 0。没有改动

    查找发现： 已手动将g_enalbe改成true，可以进controlthread的entry，但是g_motor.MotorState为OFF LINE，应该是因为M2006::MotorStateTypedef M2006::AliveCheck()里面AliveFlag == Pre_AliveFlag所以返回OFF_LINE的状态，并且 // TODO: MotorState = MOTOR_OFFLINE；忘删代码了。。。
   
    不行，改了还是OFF_LINE，watch发现 g_motor.AliveFlag一直都是0，

    IER = 2 接收中断使能没问题
    断点发现不会进can回调，ESR=16253015，TSR=2181038080，RF0R=0，表示发送错误计数器溢出而进入了Bus-Off状态， 接收 FIFO 空的
    逐次读发现ESR₁ = 8918979 = 0x008817C3，一帧一帧发、每帧都失败
    是板子物理层面的问题，发现c板没接电源（笑

    接上电源后依然进不了回调，断开C板和CAN的连接依然esr=8912979,tsr=469762057,rf0r=0，查配置，发现引脚配错了，CAN1 的发送引脚配成了 PA12，但 C 板上 CAN1_TX 实际接的是 PD1。
    改完之后可以进can回调，MotorState终于ONLINE
    
    但是电机依然不转，可能是onlineCut一直积累不到kArmTicks，onlineCut有积累但一段时间后会被清零。所以，在M2006.hpp AliveCheck()增加防抖，连续 100 拍（≈100ms）都没帧才判定为真掉线，同时 controlthread.cpp const bool linkOk=(g_motor.MotorState==DJIMotor::MOTOR_ONLINE);//先不判断&&(lostCnt==0)，给点抖动空间;

    改动完后电机可以转动，但是大约3s又停下了。

