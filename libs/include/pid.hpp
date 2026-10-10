//
// Created by cosmosmount on 2025/8/29.
//

#ifndef RM26_PID_HPP
#define RM26_PID_HPP

#include "math.hpp"
#include <cstdint>
#include<cmath>
#include "main.h"


/**
 * @brief PID模式
 */
enum PidModeType
{
    PID_POSITION = 0x01, // 位置式 PID
    PID_DELTA = 0x02,    // 增量式 PID
};

/**
 * @brief PID类
 */
class PID
{
public:
    uint8_t mode;

    float kp;
    float ki;
    float kd;

    float ref;
    float fdb;
    float err[3]{};

    float pResult;
    float iResult;
    float dResult;
    float result;

    float maxOut;
    float maxIOut;

    PID(float kp, float ki, float kd, float maxOut, float maxIOut, int mode = PID_POSITION);
    void Tuning(float tuning_kp, float tuning_ki, float tuning_kd);
    void UpdateResult();
    void Clear();
};

#endif //RM26_PID_HPP
