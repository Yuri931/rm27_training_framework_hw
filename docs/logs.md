##0.灯效信息：
1. 颜色分辨"谁出问题"，频率分辨"严重程度"：正常 = 慢闪心跳，异常 = 快闪，致命 = 常亮。

初始化没过：蓝灯常亮
imu丢失，蓝灯快闪
电机掉线，红灯快闪
2. 三态语义：不亮 = 没跑/没上电；常亮 = 卡死或初始化没过；闪烁 = 运行中。
3. 上电自检跑马灯：三色依次亮一遍，确认 LED 本身正常（也顺便验证了 MX_GPIO_Init 和引脚配置），然后再进入正常指示。
4. 错误仲裁：多个错误同时存在时只显示优先级最高的那一个，不要三个灯一起乱闪——那样什么信息都读不出来。

 注意controlthread里面volatile bool  g_enable   = false;，要在调试器里手动 ControlThread::SetEnable(true)，或者先把目标改成 0。

 单位：加速度 imu.acc_data.x/y/z —— 单位 m/s²
 陀螺仪 imu.gyro_data.x/y/z —— 单位 rad/s
 温度 imu.acc_data.temperature —— 单位 °C

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

    观测发现positionFdb=1.7左右，并没有达到设定值，current=135是数值是正常输出环路可以正常输出，但是电机不动，用手拨动电机，fdb变化，但很小。
    可能是因为：35 大约是 0.13A。这么小的电流克服不了齿轮箱的静摩擦,电机就停在半路。现在是纯 P 控制没有积分:电流正比于误差。电机一旦停下,误差就定格在 1.42,电流就定格在 135。

    先尝试给速度环加积分， maxCurrent = 3000; speedPid.Tuning(10.0f, 5.0f, 0.0f);
    现象是电机很快动起来，但是一直在两个方向快速转不会停下来，是不是一直没有达到设定位置，积分好像给得太过了，想暂定看参数电机一下子往一个方向快速好几圈转动然后停下，现在positionFdb大约是3.3。应该是两处同积分能占的额度太大。

    下面先调速度环,但是现在调试平台用不了，先放一下


接着调试IMU反馈读取问题

数据手册 6.1.2 节原文：

▎ "to read the accelerometer values in SPI mode, the user has to read 7 bytes, starting from address 0x12 (ACC data). From these bytes the user must discard the first byte and finds the acceleration information in byte #2 - #7."

6.1.1 节（陀螺仪）：

可能原因：▎ "For single byte read as well as write operations, 16-bit protocols are used... Bit #8-15: when in read mode, these are the data SDO, which are read from the address."
▎ "This dummy byte procedure does not apply to the gyroscope part."

也就是：加速度计丢弃 1 个、陀螺仪丢弃 0 个。而 BMI088.cpp:251-252 和 :302 写的是加速度计丢 2 个、陀螺仪丢 1 个——两个都多丢了 1 个字节。

为什么是 1 而不是 2：ReadReg 是"先 HAL_SPI_Transmit 发地址，再 HAL_SPI_Receive 收数据"的两段式。HAL_SPI_Transmit 结束时会 SPI_EndRxTxTransaction（等 BSY 清零）然后 __HAL_SPI_CLEAR_OVRFLAG（读 DR + SR），发地址那一个字节周期收到的数据已经被 HAL 清掉了，不会进 rx[]。所以 rx[0] 就是"地址之后的第一字节"：

不对实践表明原来的rx就是从下标2开始有地址的，

现在不知道怎么acc地址就读得没问题了（

检查发现是引脚问题，片选确实没问题，但是 MOSI 接错引脚了，将PA7配成了PB5（当时是cubeMX默认的），所以说IMU 从来没有收到过任何一条指令 —— 寄存器地址根本没送到芯片手里,它不知道你要读哪个寄存器,吐回来的自然都是垃圾。"陀螺仪全是 0xFF = MISO 一直是空闲高电平,没有任何芯片在驱动它" —— 同一个原因。片选拉低了,但芯片从没收到过有效地址,凭什么驱动 MISO。加速度计一直停在上电默认的挂起模式

泪目了就是这个问题，补上前面丢字节的，实际测得是：const uint16_t extraBytes =
        (cs == BMI088_CS_ACC) ? 1U : 0U;

现在初始化没问题，但是acc，gyor接收到的值全是0，可能是:配置根本没写进去，BMI088 的两颗 die 上电默认都是挂起(suspend)模式:
接下来用这段确认配置有没有真正写进芯片，看写进去的和读出来的是否一致
    /* ---- 临时诊断:Config 之后立刻回读,看配置到底写进去没有 ---- */
    imu.ReadReg(BMI088::BMI088_CS_ACC,  0x7C, &cfg_readback[0], 1);
    imu.ReadReg(BMI088::BMI088_CS_ACC,  0x7D, &cfg_readback[1], 1);
    imu.ReadReg(BMI088::BMI088_CS_ACC,  0x40, &cfg_readback[2], 1);
    imu.ReadReg(BMI088::BMI088_CS_ACC,  0x41, &cfg_readback[3], 1);
    imu.ReadReg(BMI088::BMI088_CS_ACC,  0x03, &cfg_readback[4], 1);
    imu.ReadReg(BMI088::BMI088_CS_ACC,  0x00, &cfg_readback[5], 1);
    imu.ReadReg(BMI088::BMI088_CS_GYRO, 0x11, &cfg_readback[6], 1);
    imu.ReadReg(BMI088::BMI088_CS_GYRO, 0x0F, &cfg_readback[7], 1);
    imu.ReadReg(BMI088::BMI088_CS_GYRO, 0x10, &cfg_readback[8], 1);
    imu.ReadReg(BMI088::BMI088_CS_GYRO, 0x00, &cfg_readback[9], 1);

    读到：0 0 171 1 16 30 0 0 129 15 ，ACC 第二个电源开关 ✗ 没写进去（期望0x04）
    acc电源没开，因为BMI088 的加速度计特性:软复位之后,紧接的第一条操作会被芯片忽略，打开电源的代码被丢弃了，在BMI088.cpp void cBMI088::Config() 软启后加入
    ReadReg(BMI088_CS_ACC, ACC_CHIP_ID_ADDR, &dummy, 1);//先丢一个
        tx_thread_sleep(1);


