#ifndef __PID_H__
#define __PID_H__

typedef enum
{
  POSITION, // 位置式
  DELTA     // 增量式
} PID_Mode;

typedef struct
{
  PID_Mode mode;

  float kp;
  float ki;
  float kd;

  float set; // 目标值
  float now; // 当前值

  float err[3];

  float Pout;
  float Iout;
  float Dout;

  float out;
  float max_iout;
  float max_out;

} PID;

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
void PID_Init(PID *pid, PID_Mode mode, float maxout, float max_iout, float kp, float ki, float kd);

/**
 * @brief 根据当前值与设定值进行PID计算
 * @param pid PID结构体
 * @param now 当前值
 * @param set 目标值/设定值
 * @retval PID计算出的控制量
 */
float PID_Calculate(PID *pid, float fdb, float set);

#endif
