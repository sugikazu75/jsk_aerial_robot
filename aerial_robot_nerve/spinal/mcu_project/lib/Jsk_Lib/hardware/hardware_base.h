/*
******************************************************************************
* File Name          : hardware_base.h
* Description        : hardware interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __HARDWARE_BASE_H
#define __HARDWARE_BASE_H

#ifndef SIMULATION
#include "config.h"
#include <ros.h>
#else
#include <ros/ros.h>
#endif

#include <math/AP_Math.h>
#include <vector>

#ifndef SIMULATION
#if NERVE_COMM
#include <Spine/spine.h>
#endif
#include "battery_status/battery_status.h" /* battery status */
#include "dshot_esc/dshot.h" /* dshot esc */
#endif
#include <spinal/ESCTelemetry.h>
#include <spinal/ESCTelemetryArray.h>
#include <spinal/Pwms.h>
#include <spinal/PwmTest.h>
#include <spinal/PwmInfo.h>

#define IDLE_DUTY 0.5f

#define MAX_PWM 1.0f // duty

#define MAX_MOTOR_NUMBER 10

#define PWM_PUB_INTERVAL 100 //100ms

class HardwareBase
{
public:
  HardwareBase();
  ~HardwareBase(){}

#ifdef SIMULATION
  void init(ros::NodeHandle* nh);
#else
  void init(TIM_HandleTypeDef* htim1, TIM_HandleTypeDef* htim2,
            DShot* dshot, BatteryStatus* bat, ros::NodeHandle* nh, osMutexId* mutex = NULL);
#endif
  void update();
  float getPwm(uint8_t index) {return target_pwm_[index];}

private:
#ifndef SIMULATION
  TIM_HandleTypeDef* pwm_htim1_;
  TIM_HandleTypeDef* pwm_htim2_;
  BatteryStatus* bat_;
  DShot* dshot_;
  osMutexId* mutex_;
#endif

  ros::NodeHandle* nh_;
  ros::Publisher esc_telem_pub_;
  ros::Publisher pwms_pub_;
  spinal::ESCTelemetryArray esc_telem_msg_;
  spinal::Pwms pwms_msg_;


#ifdef SIMULATION
  ros::Subscriber pwm_info_sub_;
  ros::Subscriber pwm_test_sub_;
#else
  ros::Subscriber<spinal::PwmInfo, HardwareBase> pwm_info_sub_;
  ros::Subscriber<spinal::PwmTest, HardwareBase> pwm_test_sub_;
#endif

  bool start_control_flag_;
  uint8_t motor_number_;
  uint32_t voltage_update_last_time_;

  // thrust
  float target_thrust_[MAX_MOTOR_NUMBER];
  float force_landing_thrust_;

  // pwm
  float target_pwm_[MAX_MOTOR_NUMBER];
  float pwm_test_value_[MAX_MOTOR_NUMBER]; // PWM Test
  bool pwm_test_flag_;
  float min_duty_;
  float min_thrust_;
  float max_duty_;
  int8_t pwm_conversion_mode_;
  std::vector<spinal::MotorInfo> motor_info_;
  uint32_t pwm_pub_last_time_;
  uint8_t motor_ref_index_;
  float v_factor_;
  int8_t rotor_devider_;

  // voltage
  float sim_voltage_;

  void pwmInfoCallback(const spinal::PwmInfo &info_msg);
  void pwmTestCallback(const spinal::PwmTest& pwm_msg);

  void baseInit();
  void reset();
  void publish();
  void pwmsControl();
  void pwmConversion();
  void setMotorNumber(uint8_t motor_number);

  void setPwmTestMode(bool pwm_test_flag){pwm_test_flag_ = pwm_test_flag; }

};

#endif
