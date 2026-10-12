#ifndef THREADS_HPP
#define THREADS_HPP
#include"tx_api.h"                   // 这个头文件会被 app_threadx.c（C 文件）包含并调用，
                                       //而 Threads_Init 的定义在 .cpp 里（C++ 会改名），
                                      // 所以必须用 extern "C" 声明成 C 链接，否则链接时报 undefined reference

#ifdef __cplusplus
extern "C" {
#endif

extern TX_SEMAPHORE sem_init_ok;
extern TX_SEMAPHORE sem_imu;         //BMI088线程心跳
extern TX_SEMAPHORE sem_motor;

void AliveThread_Entry(ULONG thread_input);
void ImuThread_Entry(ULONG thread_input);   
void MotorThread_Entry(ULONG thread_input);

void Threads_Init(TX_BYTE_POOL *byte_pool);           


#ifdef __cplusplus
}
#endif
#endif

