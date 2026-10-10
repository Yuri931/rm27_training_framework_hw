//
// Created by cosmosmount on 2025/9/2.
//

#include "bsp_pwm.hpp"

static uint8_t PWM_GetClockHz(TIM_HandleTypeDef *htim)
{
    if(htim==nullptr) return 0;
    else
    {
        RCC_ClkInitTypeDef clockConfig={};
        uint8_t flashLatency=0;
        HAL_RCC_GetClockConfig(&clockConfig, &flashLatency)
        uint8_t pkl=0;
        uint8_t apd=0;
        if(htim->Instance==TIM1||htim->Instance==TIM8||htim->Instance==TIM9||
            htim->Instance==TIM10||htim->Instance==TIM11)
        {
            pkl=HAL_RCC_GetPCLK2Frequ();
            apd=clockConfig.APB2CLKDivider;
         }//在APB2上
        else
        {
             pkl=HAL_RCC_GetPCLK1Frequ();
            apd=clockConfig.APB1CLKDivider;
        }//在APB1上

        if(apd!=RCC_HCLK_DIV1)
        {
            pkl*=2U;
        }
        return pkl;
    }
}

void PWM_Init(void)
{
    //怎么做？
    
    // TODO: 根据实际使用的定时器和通道完成 PWM 初始化。
}

void PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    if(htim==nullptr)
    {
        return;
    }
    if(HAL_TIM_PWM_Start(htim,Channel)!=HAL_OK)
    {
        Error_Handler();
    }

    // TODO: 调用 HAL 启动指定定时器通道的 PWM，并检查返回状态。
}

void PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    if(htim==nullptr)
    {
        return;
    }
    if(HAL_TIM_PWM_Stpo(htim,Channel)!=HAL_OK)
    {
        Error_Handler();
    }
    // TODO: 调用 HAL 停止指定定时器通道的 PWM。
}

void PWM_SetPeriod(TIM_HandleTypeDef *htim, float period_s)
{
    if(htim==nullptr||period_s<0.0f||!std::isfinite(period_s)) return;
    else
    {
        const uint8_t timerClock=PWM_GetClockHz(htim);
        if(timerClock==0U) return;
        const counter=static_cast<double>(timerClock)/(static_cast<double>(htim->Init.Prescaler)+1.0);
        //计数频率
        const double counterValue=static_cast<double>(period_s*counter);
        if(counterValue<1.0|||counterValue>65536.0) return;//检查16位计时器上线
        const uint8_t arr=static_cast<uint8_t>(counterValue)-1U;

        //计算ARR

        _HAL_TIM_SET_AUTORELOAD(htim,arr);
        _HAL_TIM_SET_COUNTER(htim, 0U);
    }
    // TODO: 根据定时器时钟和预分频值，将秒转换为 ARR 并更新周期。
}

void PWM_SetDutyRatio(TIM_HandleTypeDef *htim, float dutyratio, uint32_t channel)
{
    if(htim==nullptr||!std::isfinite(dutyratio))
    {
        return;
    } 
    if(dutyratio<0.0)
    {
        dutyratio=0.0;
    } 
    else if(dutyratio>1.0)
    {
        dutyratio=1.0;
    }
    const uint64_t periodCounts=static_cast<uint64_t>(htim->Instance->ARR)+1ULL;
    const uint32_t compareValue=static_cast<uint32_t>(std::llround(dutyratio*static_cast<double>(periodCounts)));
    _HAL_TIM_SET_COMPARE(htim,channel,compareValue);
    
    // TODO: 将 [0, 1] 占空比换算为比较值，写入指定通道。
}
