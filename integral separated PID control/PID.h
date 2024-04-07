#ifndef __PID_H
#define __PID_H

// #include "sys.h"

enum
{
  LAST  = 0,
  NOW   = 1,
};

typedef struct _pid_struct_t
{
  float kp;
  float ki;
  float kd;
  float i_max;
  float out_max;
  float i_discrete;
  
  float ref;      // target value
  float fdb;      // feedback value
  float err[2];   // error and last error

	float set;
  float get;
	
  float p_out;
  float i_out;
  float d_out;
  float output;
}pid_struct_t;

extern pid_struct_t pid_wheel_spd[4];
extern pid_struct_t pid_chassis_angle;
void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max,
              float i_discrete);


float mapTo01(float variable, float min_value, float max_value);

float pid_general(pid_struct_t *pid, float get, float set);
							
#endif          

