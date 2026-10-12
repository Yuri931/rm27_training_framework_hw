#include "threads.hpp"
#include "LED.hpp"
#include "BMI088.hpp"
#include "DJIMotorHandler.hpp"
TX_SEMAPHORE sem_init_ok;
TX_SEMAPHORE sem_imu;         //BMI088线程心跳,信号量
TX_SEMAPHORE sem_motor;

static TX_THREAD alive_thread;    //三个线程
static TX_THREAD imu_thread;
static TX_THREAD motor_thread;



static bool TakeHeartbeat(TX_SEMAPHORE *sem)
{
    bool got=false;
    while(tx_semaphore_get(sem,TX_NO_WAIT)==TX_SUCCESS)      //取信号量
    {
        got=true;
    }
    return got;
}

void Threads_Init(TX_BYTE_POOL *byte_pool)
{
    VOID *stack=TX_NULL;
    tx_semaphore_create(&sem_init_ok,(CHAR*)"sem_init_ok",0);
    tx_semaphore_create(&sem_imu,(CHAR*)"sem_imu",0);
    tx_semaphore_create(&sem_motor,(CHAR*)"sem_motor",0);

    if(tx_byte_allocate(byte_pool,&stack,1024,TX_NO_WAIT)==TX_SUCCESS)
    {
        tx_thread_create(&alive_thread,(CHAR*)"alive_thread",AliveThread_Entry,0,stack,1024,10,10,TX_NO_TIME_SLICE,TX_AUTO_START);
                                                                       //抢占优先级怎么定
    }
    if(tx_byte_allocate(byte_pool,&stack,2048,TX_NO_WAIT)==TX_SUCCESS)
    {
        tx_thread_create(&imu_thread,(CHAR*)"imu_thread",ImuThread_Entry,0,stack,2048,5,5,TX_NO_TIME_SLICE,TX_AUTO_START);
    }
    if(tx_byte_allocate(byte_pool,&stack,2048,TX_NO_WAIT)==TX_SUCCESS)
    {
        tx_thread_create(&motor_thread,(CHAR*)"motor_thread",MotorThread_Entry,0,stack,2048,6,6,TX_NO_TIME_SLICE,TX_AUTO_START);
    }
}

void AliveThread_Entry(ULONG thread_input)
{
    (void)thread_input;
    bool init_ok=false;
    uint8_t imu_lost=0;
    uint8_t motor_lost=0;
    ULONG tick;
     LED_Set(LED_Color::Red, true);   tx_thread_sleep(200); LED_Set(LED_Color::Red, false);
    LED_Set(LED_Color::Green, true); tx_thread_sleep(200); LED_Set(LED_Color::Green, false);
    LED_Set(LED_Color::Blue, true);  tx_thread_sleep(200); LED_Set(LED_Color::Blue, false);

    for(;;)
    {
        if(!init_ok&&tx_semaphore_get(&sem_init_ok,TX_NO_WAIT)==TX_SUCCESS)
        {
            //这个线程只会从 sem_init_ok 里取一次令牌。
               //如果系统里还有别的线程也在等这个信号量，或者初始化过程中 sem_init_ok 被 put 了多次（比如意外多次触发），这个 if 保证它只响应第一次，避免重复触发。      
            init_ok=true;
            tx_thread_sleep(1000); //为什么要等宽限期1s？线程主动让出 CPU，去干别的活。等 1 秒后再回来检查一次
        }
        if(!init_ok)
        {
            LED_Set(LED_Color::Blue,true);
             LED_Set(LED_Color::Green,false);
            LED_Set(LED_Color::Red, false);
            tx_thread_sleep(100);
            continue;          //跳过后面的重新循环，在初始化完成之前不会取心跳
        }
        if(TakeHeartbeat(&sem_imu))
        {
            imu_lost=0;
        }
        else if(imu_lost<3)     //没取到心跳但是防抖
        {
            imu_lost++;
        }
        if(TakeHeartbeat(&sem_motor))
        {
            motor_lost=0;
        }
        else if(motor_lost<3)     //防抖
        {
            motor_lost++;
        }
        tick++;
        const bool slow=((tick/5)%2)==0;   //慢闪烁，5个tick翻转一次
        const bool fast=((tick/2)%2)==0;   //快闪烁，2个tick翻转一次
        if(imu_lost>=3)                         //BMI088掉线
        {
             LED_Set(LED_Color::Green, false);
            LED_Set(LED_Color::Red,   false);
            LED_Set(LED_Color::Blue,  fast);
        }
        else if(motor_lost>=3)                   //电机掉线
        {
            LED_Set(LED_Color::Green, false);
            LED_Set(LED_Color::Blue,  false);
            LED_Set(LED_Color::Red,   fast);
        }
        else                                 //正常
        {
            LED_Set(LED_Color::Green, slow);
            LED_Set(LED_Color::Blue,  false);
            LED_Set(LED_Color::Red,   false);
        }
        tx_thread_sleep(100);               //1tick=1ms
    }
}
