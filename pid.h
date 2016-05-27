#ifndef pid
#define pid

#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <math.h>

class PID {

private:
  //PID constants
  float m_Kp;
  float m_Ki;
  float m_Kd;

  //PID constants
  float m_err;
  float m_last_err;
  float m_sum_err;
  float m_ddt_err;
  float m_lastInput;
  float m_outmax;
  float m_outmin;
  float m_summax;
  float m_summin;

  float m_output;

public:
  PID();
  PID(float,float,float);
  float update_pid_std(float setpt, float input, float dt,int thr);
  void  updateKpKi(float setpt, float input);
  void  set_Kpid(float, float, float);
  void  set_windup_bounds(float, float);
  void  reset();
  float get_kp();
  float get_ki();
  float get_kd();
  float setpoint;

  float get_kp_coeff();
  float get_ki_coeff();
  float get_kd_coeff();
  void set_kp_coeff(float kp_);
  void set_ki_coeff(float ki_);
  void set_kd_coeff(float kd_);
};

extern PID yprSTAB[3];
extern PID yprRATE[3];

#endif
