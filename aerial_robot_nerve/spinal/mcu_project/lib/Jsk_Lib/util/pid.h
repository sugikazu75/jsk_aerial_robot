/*
******************************************************************************
* File Name          : pid.h
* Description        : PID Controller
* Author             : K.Sugihara (2025/3/27)
******************************************************************************
*/

#ifndef __PID_H__
#define __PID_H__

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

class PID
{
public:
  PID():
    result_(0), p_gain_(0), i_gain_(0), d_gain_(0)
  {}

  virtual void update(const float err_p, const float du, const float err_d)
  {
    err_p_ = clamp(err_p, -limit_err_p_, limit_err_p_);
    err_i_prev_ = err_i_;
    err_i_ = clamp(err_i_ + err_p_ * du, -limit_err_i_, limit_err_i_);
    err_d_ = clamp(err_d, -limit_err_d_, limit_err_d_);

    p_term_ = clamp(err_p_ * p_gain_, -limit_p_, limit_p_);
    i_term_ = clamp(err_i_ * i_gain_, -limit_i_, limit_i_);
    d_term_ = clamp(err_d_ * d_gain_, -limit_d_, limit_d_);

    result_ = clamp(p_term_ + i_term_ + d_term_, -limit_sum_, limit_sum_);
  }

  void setPGain(const double p_gain) { p_gain_ = p_gain; }
  void setIGain(const double i_gain) { i_gain_ = i_gain; }
  void setDGain(const double d_gain) { d_gain_ = d_gain; }
  void setGains(const double p_gain, const double i_gain, const double d_gain)
  {
    setPGain(p_gain);
    setIGain(i_gain);
    setDGain(d_gain);
  }

private:
  float result_;
  float p_gain_, i_gain_, d_gain_;
  float p_term_, i_term_, d_term_;
  float err_p_, err_i_, err_i_prev_, err_d_;
  float limit_sum_, limit_p_, limit_i_, limit_d_;
  float limit_err_p_, limit_err_i_, limit_err_d_;

  void clamp(float value, float min_limit, float max_limit)
  {
    return value < max_limit ? (min_limit < value ? value : min_limit) : max_limit;
  }

};


#endif
