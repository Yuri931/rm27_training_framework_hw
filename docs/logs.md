灯效信息：
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

gyro1 = 00 FF FF FF
gyro2 = FF FF FF

注意这里连 0x80 都没有——所以也不是首读无效的问题（首读无效会返回 0x80 这种具体值）。这个是陀螺根本没应答：要么没被选中，要么它压根不在 SPI 模式。

先做这个实验：把 SPI 时钟降下来

你的 SPI_BAUDRATEPRESCALER_16 = 168/16 = 10.5 MHz，超了 BMI088 手册的 10 MHz 上限。"响应稳定地晚一个字节"和"某个 die 完全不响应"都符合超频跑在边缘上的表现——这可以同时解释你这两个现象。

频率168/16>10超时了，改动：hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;