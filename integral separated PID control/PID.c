#include "pid.h"

// // 底盘电机 PID 结构体定义
// pid_struct_t pid_wheel_spd[4] = {0};
// pid_struct_t pid_chassis_angle = {0};

/**
  * @brief  init pid parameter
  * @param  pid struct
    @param  parameter
  * @retval None
*/
static void abs_limit(float *a, float ABS_MAX)
{
	if (*a > ABS_MAX)
		*a = ABS_MAX;
	if (*a < -ABS_MAX)
		*a = -ABS_MAX;
}
void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max,
              float i_discrete)
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
  pid->i_discrete = i_discrete;
}

/**
  * @brief  位置式PID函数
  * @param  pid结构体
    @param  反馈值
    @param  目标值
  * @retval 计算输出值
  */

float mapTo01(float variable, float min_value, float max_value) {
    float mapped_value = (variable - min_value) / (max_value - min_value);
    return mapped_value;
}

float pid_general(pid_struct_t *pid, float get, float set)
{
	pid->get = get;
	pid->set = set;
	pid->err[NOW] = set - get;

  float mapped_err = mapTo01(pid->err[NOW], 0, pid->i_discrete);

  if (mapped_err > 1.0) {
    mapped_err = 1.0;
  }

	pid->p_out = pid->kp * pid->err[NOW];
	pid->i_out += pid->ki * pid->err[NOW] *(1 - mapped_err);
	pid->d_out = pid->kd * (pid->err[NOW] - pid->err[LAST]);

	abs_limit(&(pid->i_out), pid->i_max);
	pid->output = pid->p_out + pid->i_out + pid->d_out;
	abs_limit(&(pid->output), pid->out_max);

	pid->err[LAST] = pid->err[NOW];

	return pid->output;
}
