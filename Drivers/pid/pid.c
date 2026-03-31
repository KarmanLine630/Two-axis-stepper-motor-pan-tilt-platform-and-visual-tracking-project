#include "pid.h"
#include "string.h"

/**
 * @brief 限制数据大小
 * @param input 需要限制的数据
 * @param max 数据最大值
 * @retval 若数据超出+-max则返回+-max,否则返回input
 */
static float LimitMax(float input, float max)
{
    if (input > max)
        input = max;
    else if (input < -max)
        input = -max;
}

/**
 * @brief PID结构体初始化
 * @param pid PID结构体
 * @param mode 位置式POSITION或者增量式DELTA
 * @param maxout 输出的最大值
 * @param max_iout 积分输出的最大值
 * @param kp 比例系数Kp
 * @param ki 积分系数Ki
 * @param kd 微分系数Kd
 */
void PID_Init(PID *pid, PID_Mode mode, float maxout, float max_iout, float kp, float ki, float kd)
{
    if (pid == NULL)
    {
        return;
    }
    memset(pid, 0, sizeof(PID));
    pid->mode = mode;
    pid->max_iout = max_iout;
    pid->max_out = maxout;
    pid->kp = kp;
    pid->kd = kd;
    pid->ki = ki;
}

/**
 * @brief 根据当前值与设定值进行PID计算
 * @param pid PID结构体
 * @param now 当前值
 * @param set 目标值/设定值
 * @retval PID计算出的控制量
 */
float PID_Calculate(PID *pid, float now, float set)
{
    if (pid == NULL)
        return 0;

    pid->err[2] = pid->err[1];
    pid->err[1] = pid->err[0];
    pid->set = set;
    pid->now = now;
    pid->err[0] = set - now;
    if (pid->mode == POSITION)
    {
        pid->Pout = pid->kp * pid->err[0];
        if (pid->err[0] < 300 && pid->err[0] > -300)
            pid->Iout += pid->ki * pid->err[0];
        pid->Dout = pid->kd * (pid->err[0] - pid->err[1]);
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == DELTA)
    {
        pid->Pout = pid->kp * (pid->err[0] - pid->err[1]);
        pid->Iout = pid->ki * pid->err[0];
        pid->Dout = pid->kd * (pid->err[0] - 2.0f * pid->err[1] + pid->err[2]);
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}
