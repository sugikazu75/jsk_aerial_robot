
#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "hardware/hardware_base.h"


#ifdef SIMULATION
HardwareBase::HardwareBase() : sim_voltage_(0)
{}

void HardwareBase::init(ros::NodeHandle* nh)
{
  nh_ = nh;

  pwms_pub_ = nh_->advertise<spinal::Pwms>("motor_pwms", 1);
  pwm_info_sub_ = nh_->subscribe("motor_info", 1, &HardwareBase::pwmInfoCallback, this);
  pwm_test_sub_ = nh_->subscribe("pwm_test", 1, &HardwareBase::pwmTestCallback, this);
  sim_vol_sub_ = nh_->subscribe("set_sim_voltage", 1, &HardwareBase::setSimVolCallback, this);

  baseInit();
}

#else

HardwareBase::HardwareBase():
  pwms_pub_("motor_pwms", &pwms_msg_),
  esc_telem_pub_("esc_telem", &esc_telem_msg_),
  pwm_info_sub_("motor_info", &HardwareBase::pwmInfoCallback, this),
  pwm_test_sub_("pwm_test", &HardwareBase::pwmTestCallback, this)
{}

void HardwareBase::init(TIM_HandleTypeDef* htim1, TIM_HandleTypeDef* htim2,
                        DShot* dshot, BatteryStatus* bat, ros::NodeHandle* nh, osMutexId* mutex)
{
  pwm_htim1_ = htim1;
  pwm_htim2_ = htim2;
  nh_ = nh;
  dshot_ = dshot;
  bat_ = bat;
  mutex_ = mutex;

  nh_->advertise(pwms_pub_);
  nh_->advertise(esc_telem_pub_);

  if(!dshot_)
    {
      HAL_TIM_PWM_Stop(pwm_htim1_, TIM_CHANNEL_1);
      HAL_TIM_Base_Stop(pwm_htim1_);
      HAL_TIM_Base_DeInit(pwm_htim1_);

      pwm_htim1_->Init.Prescaler = 3;
      pwm_htim1_->Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
      pwm_htim1_->Init.Period = 50000;

      TIM_OC_InitTypeDef sConfigOC = {0};
      sConfigOC.OCMode = TIM_OCMODE_PWM1;
      sConfigOC.Pulse = 1000;
      sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
      sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

      while(HAL_TIM_Base_Init(pwm_htim1_) != HAL_OK);
      while(HAL_TIM_PWM_Init(pwm_htim1_) != HAL_OK);
      while(HAL_TIM_PWM_ConfigChannel(pwm_htim1_, &sConfigOC, TIM_CHANNEL_1) != HAL_OK);

      if (pwm_htim1_->hdma[TIM_DMA_ID_UPDATE] != NULL) {
        HAL_DMA_DeInit(pwm_htim1_->hdma[TIM_DMA_ID_UPDATE]);
        pwm_htim1_->hdma[TIM_DMA_ID_UPDATE] = NULL;
      }

      HAL_TIM_Base_Start(pwm_htim1_);

      HAL_TIM_PWM_Start(pwm_htim1_, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(pwm_htim1_, TIM_CHANNEL_2);
      HAL_TIM_PWM_Start(pwm_htim1_, TIM_CHANNEL_3);
      HAL_TIM_PWM_Start(pwm_htim1_, TIM_CHANNEL_4);
    }

  HAL_TIM_PWM_Start(pwm_htim2_,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(pwm_htim2_,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(pwm_htim2_,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(pwm_htim2_,TIM_CHANNEL_4);

  baseInit();
}
#endif

void HardwareBase::baseInit()
{
  // pwm
  pwm_conversion_mode_ = -1;
  min_duty_ = IDLE_DUTY;
  max_duty_ = IDLE_DUTY; //should assign right value from PC(ros)
  min_thrust_ = 0;
  force_landing_thrust_ = 0;
  pwm_pub_last_time_ = 0;
  pwm_test_flag_ = false;
  pwm_pub_last_time_ = 0;

  // voltage
  motor_info_.resize(0);
  v_factor_ = 1;
  motor_ref_index_ = 0;
  voltage_update_last_time_ = 0;

  reset();
}

void HardwareBase::reset()
{
  for(int i = 0; i < MAX_MOTOR_NUMBER; i++)
    {
      target_pwm_[i] = IDLE_DUTY;
      pwm_test_value_[i] = IDLE_DUTY;
    }
}

void HardwareBase::publish()
{
#ifdef SIMULATION
  if(HAL_GetTick() - pwm_pub_last_time_ > PWM_PUB_INTERVAL)
    {
      pwm_pub_last_time_ = HAL_GetTick();
      pwms_pub_.publish(pwms_msg_);
    }
#else
  if(HAL_GetTick() - pwm_pub_last_time_ > PWM_PUB_INTERVAL)
    {
      pwm_pub_last_time_ = HAL_GetTick();
      pwms_pub_.publish(&pwms_msg_);
    }
#endif
}

void HardwareBase::update()
{
}

void HardwareBase::pwmsControl(void)
{
  /* target thrust -> target pwm */
  pwmConversion();

#ifndef SIMULATION
  /* nerve comm type */
#if NERVE_COMM
  for(int i = 0; i < motor_number_; i++) {
    Spine::setMotorPwm(target_pwm_[i] * 2000 - 1000, i);
  }
#endif

  if(dshot_)
    {
      /* direct pwm type */
      uint16_t motor_value[4] = { 0, 0, 0, 0 };
      for (int i = 0; i < 4; i++)
        {
          // target_pwm_: 0.5 ~ 1.0
          uint16_t motor_v = (uint16_t)((target_pwm_[i] - 0.5) / 0.5 * DSHOT_RANGE + DSHOT_MIN_THROTTLE);

          if (motor_v > DSHOT_MAX_THROTTLE)
            motor_v = DSHOT_MAX_THROTTLE;
          else if (motor_v < DSHOT_MIN_THROTTLE)
            motor_v = DSHOT_MIN_THROTTLE;

          motor_value[i] = motor_v;
        }

      dshot_->write(motor_value, dshot_->is_telemetry_);

      if (dshot_->is_telemetry_)
        {
          if (dshot_->esc_reader_.is_update_all_msg_)
            {
              esc_telem_msg_.stamp = nh_->now();
              esc_telem_msg_.esc_telemetry_1 = dshot_->esc_reader_.esc_msg_1_;
              esc_telem_msg_.esc_telemetry_2 = dshot_->esc_reader_.esc_msg_2_;
              esc_telem_msg_.esc_telemetry_3 = dshot_->esc_reader_.esc_msg_3_;
              esc_telem_msg_.esc_telemetry_4 = dshot_->esc_reader_.esc_msg_4_;
              esc_telem_pub_.publish(&esc_telem_msg_);

              float voltage_ave = (float)(dshot_->esc_reader_.esc_msg_1_.voltage + dshot_->esc_reader_.esc_msg_2_.voltage +
                                          dshot_->esc_reader_.esc_msg_3_.voltage + dshot_->esc_reader_.esc_msg_4_.voltage) / 400.0;
              bat_->update(voltage_ave);

              dshot_->esc_reader_.is_update_all_msg_ = false;
            }
        }
    }
  else
    {
      pwm_htim1_->Instance->CCR1 = (uint32_t)(target_pwm_[0] * pwm_htim1_->Init.Period);
      pwm_htim1_->Instance->CCR2 = (uint32_t)(target_pwm_[1] * pwm_htim1_->Init.Period);
      pwm_htim1_->Instance->CCR3 = (uint32_t)(target_pwm_[2] * pwm_htim1_->Init.Period);
      pwm_htim1_->Instance->CCR4 = (uint32_t)(target_pwm_[3] * pwm_htim1_->Init.Period);
    }

  pwm_htim2_->Instance->CCR1 = (uint32_t)(target_pwm_[4] * pwm_htim2_->Init.Period);
  pwm_htim2_->Instance->CCR2 = (uint32_t)(target_pwm_[5] * pwm_htim2_->Init.Period);
  pwm_htim2_->Instance->CCR3 = (uint32_t)(target_pwm_[6] * pwm_htim2_->Init.Period);
  pwm_htim2_->Instance->CCR4 = (uint32_t)(target_pwm_[7] * pwm_htim2_->Init.Period);
#endif
}

void HardwareBase::pwmConversion()
{
  auto convert = [this](float target_thrust)
    {
      float scaled_thrust = v_factor_ * target_thrust / rotor_devider_;
      float target_pwm = 0;
      if (scaled_thrust < 0) scaled_thrust = 0;

      switch(pwm_conversion_mode_)
        {
        case spinal::MotorInfo::SQRT_MODE:
          {
            /* pwm = F_inv[(V_ref / V)^2 f] */
            float sqrt_tmp = motor_info_[motor_ref_index_].polynominal[1] * motor_info_[motor_ref_index_].polynominal[1] - 4 * 10 * motor_info_[motor_ref_index_].polynominal[2] * (motor_info_[motor_ref_index_].polynominal[0] - scaled_thrust); //special decimal order shift (x10)
            target_pwm = (-motor_info_[motor_ref_index_].polynominal[1] + sqrt_tmp * ap::inv_sqrt(sqrt_tmp)) / (2 * motor_info_[motor_ref_index_].polynominal[2]);
            break;
          }
        case spinal::MotorInfo::POLYNOMINAL_MODE:
          {
            /* pwm = F_inv[(V_ref / V)^1.5 f] */
            float tenth_scaled_thrust = scaled_thrust * 0.1f; //special decimal order shift (x0.1)
            /* 4 dimensional */
            int max_dimenstional = 4;
            target_pwm = motor_info_[motor_ref_index_].polynominal[max_dimenstional];
            for (int j = max_dimenstional - 1; j >= 0; j--)
              target_pwm = target_pwm * tenth_scaled_thrust + motor_info_[motor_ref_index_].polynominal[j];
            break;
          }
        default:
          {
            break;
          }
        }
      return target_pwm / 100; // target_pwm is [%]
    };

  if(pwm_test_flag_) /* motor pwm test */
    {
      for(int i = 0; i < MAX_MOTOR_NUMBER; i++)
        {
          target_pwm_[i] = pwm_test_value_[i];
        }
      return;
    }

  if(motor_info_.size() == 0) return;


  /* convert to target pwm */
  for(int i = 0; i < motor_number_; i++)
    {
      if(start_control_flag_)
        {
          target_pwm_[i] = convert(target_thrust_[i]);

          /* constraint */
          if(target_pwm_[i] < min_duty_) target_pwm_[i]  = min_duty_;
          else if(target_pwm_[i]  > max_duty_) target_pwm_[i]  = max_duty_;
        }

      /* for ros */
      pwms_msg_.motor_value[i] = (target_pwm_[i] * 2000);
    }
}

void HardwareBase::setMotorNumber(uint8_t motor_number)
{
#ifdef SIMULATION
  pwms_msg_.motor_value.resize(motor_number);
#else
  pwms_msg_.motor_value_length = motor_number;
  pwms_msg_.motor_value = new uint16_t[motor_number];
#endif
  for(int i = 0; i < motor_number; i++) pwms_msg_.motor_value[i] = 0;

  motor_number_ = motor_number;
}

void HardwareBase::pwmTestCallback(const spinal::PwmTest& pwm_msg)
{
#ifndef SIMULATION
  if(pwm_msg.pwms_length && !pwm_test_flag_)
    {
      pwm_test_flag_ = true;
      nh_->logwarn("Enter pwm test mode");
    }
  else if(!pwm_msg.pwms_length && pwm_test_flag_)
    {
      pwm_test_flag_ = false;
      nh_->logwarn("Escape from pwm test mode");
      return;
    }

  if(pwm_msg.motor_index_length)
    {
      /*Individual test mode*/
      if(pwm_msg.motor_index_length != pwm_msg.pwms_length)
        {
          nh_->logerror("The number of index does not match the number of pwms.");
          return;
        }
      for(int i = 0; i < pwm_msg.motor_index_length; i++){
        int motor_index = pwm_msg.motor_index[i];
                /*fail safe*/
        if (pwm_msg.pwms[i] >= IDLE_DUTY && pwm_msg.pwms[i] <= MAX_PWM)
          {
            pwm_test_value_[motor_index] = pwm_msg.pwms[i];
          }
        else
          {
            nh_->logwarn("FAIL SAFE!  Invaild PWM value for motor");
            pwm_test_value_[motor_index] = IDLE_DUTY;
          }
      }
    }
  else
    {
      /*Simultaneous test mode*/
      for(int i = 0; i < MAX_MOTOR_NUMBER; i++){
        /*fail safe*/
        if (pwm_msg.pwms[0] >= IDLE_DUTY && pwm_msg.pwms[0] <= MAX_PWM)
          {
            pwm_test_value_[i] = pwm_msg.pwms[0];
          }
        else
          {
            nh_->logwarn("FAIL SAFE!  Invaild PWM value for motors");
            pwm_test_value_[i] = IDLE_DUTY;
          }
      }
    }
#endif
}

void HardwareBase::pwmInfoCallback( const spinal::PwmInfo &info_msg)
{
#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexWait(*mutex_, osWaitForever);
#endif

  min_duty_ = info_msg.min_pwm;
  max_duty_ = info_msg.max_pwm;
  pwm_conversion_mode_ = info_msg.pwm_conversion_mode;

  motor_info_.resize(0);

#ifdef SIMULATION
  for(int i = 0; i < info_msg.motor_info.size(); i++)
#else
  for(int i = 0; i < info_msg.motor_info_length; i++)
#endif
    {
      motor_info_.push_back(info_msg.motor_info[i]);
    }

#ifdef SIMULATION
  if(sim_voltage_== 0) sim_voltage_ = motor_info_[0].voltage;
#endif

#ifndef SIMULATION
  /* mutex to protect the completion of following update  */
  if(mutex_ != NULL) osMutexRelease(*mutex_);
#endif
}
