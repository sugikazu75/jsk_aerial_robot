#include <aerial_robot_control/util/motor_power_publisher.h>

motorPowerPublisher::motorPowerPublisher(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh),
  nhp_(nhp)
{
  motor_pwms_sub_ = nh_.subscribe("motor_pwms", 1, &motorPowerPublisher::motorPwmsCallback, this);
  battery_voltage_sub_ = nh_.subscribe("battery_voltage_status", 1, &motorPowerPublisher::batteryVoltageStatusCallback, this);
  motor_power_pub_ = nh_.advertise<std_msgs::Float32>("motor_power", 10);

  timer_ = nh_.createTimer(ros::Duration(0.025), &motorPowerPublisher::timerCallback, this);

  rosParamInit();
}

void motorPowerPublisher::rosParamInit()
{
  ros::NodeHandle power_nh(nh_, "motor_power");
  int ref_num;
  getParam<int>(power_nh, "ref_num", ref_num, 0);
  voltages_.resize(ref_num);
  motor_infos_.resize(ref_num);

  for(int i = 0; i < ref_num; i++)
    {
      ros::NodeHandle ref_nh(power_nh, std::string("ref")+std::to_string(i + 1));
      int dim;
      getParam<double>(ref_nh, "voltage", voltages_.at(ref_num - 1 - i), 0.0);
      getParam<int>(ref_nh, "dim", dim, 0);
      motor_infos_.at(ref_num - 1 - i).resize(dim + 1);
      for(int j = 0; j < dim + 1; j++)
        {
          double polynominal;
          getParam<double>(ref_nh, std::string("polynominal") + std::to_string(j), polynominal, 0.0);
          motor_infos_.at(ref_num - 1 - i).at(j) = polynominal;
        }
    }

  bool simulation_mode;
  nh_.param("/use_sim_time", simulation_mode, false);
  if(simulation_mode)
    {
      battery_voltage_ = voltages_.at(ref_num - 1); // use max voltage in reference when simulation mode
    }
}

void motorPowerPublisher::motorPwmsCallback(const spinal::PwmsPtr& msg)
{
  motor_pwms_.resize(msg->motor_value.size());
  for(int i = 0; i < motor_pwms_.size(); i++)
    {
      motor_pwms_.at(i) = msg->motor_value.at(i);
    }
}

void motorPowerPublisher::batteryVoltageStatusCallback(const std_msgs::Float32Ptr& msg)
{
  battery_voltage_ = msg->data;
  if(battery_voltage_ < voltages_.at(0) || voltages_.at(voltages_.size() - 1) < battery_voltage_)
    {
      ROS_ERROR_STREAM("input voltage is not in references");
    }
}

void motorPowerPublisher::timerCallback(const ros::TimerEvent& e)
{
  if(battery_voltage_ < voltages_.at(0) || voltages_.at(voltages_.size() - 1) < battery_voltage_)
    return;

  int upper_voltage_index;
  int lower_voltage_index;
  for(int i = 0; i < voltages_.size(); i++)
    {
      if(battery_voltage_ <= voltages_.at(i))
        {
          upper_voltage_index = i;
          lower_voltage_index = i - 1;
        }
    }
  double pwm_percent = 0.0;
  double upper_power = 0.0;
  double lower_power = 0.0;
  for(int i = 0; i < motor_pwms_.size(); i++)
    {
      pwm_percent = motor_pwms_.at(i) / 2000.0 * 100.0;
      upper_power +=
        battery_voltage_ * (motor_infos_.at(upper_voltage_index).at(0) +
                            motor_infos_.at(upper_voltage_index).at(1) * std::pow(pwm_percent, 1) +
                            motor_infos_.at(upper_voltage_index).at(2) * std::pow(pwm_percent, 2) +
                            motor_infos_.at(upper_voltage_index).at(3) * std::pow(pwm_percent, 3) +
                            motor_infos_.at(upper_voltage_index).at(4) * std::pow(pwm_percent, 4));
      lower_power +=
        battery_voltage_ * ( motor_infos_.at(lower_voltage_index).at(0) +
                             motor_infos_.at(lower_voltage_index).at(1) * std::pow(pwm_percent, 1) +
                             motor_infos_.at(lower_voltage_index).at(2) * std::pow(pwm_percent, 2) +
                             motor_infos_.at(lower_voltage_index).at(3) * std::pow(pwm_percent, 3) +
                             motor_infos_.at(lower_voltage_index).at(4) * std::pow(pwm_percent, 4));
    }

  double power_ratio = (battery_voltage_ - voltages_.at(lower_voltage_index)) / (voltages_.at(upper_voltage_index) - voltages_.at(lower_voltage_index));
  double power = upper_power * power_ratio + lower_power * (1 - power_ratio);

  std_msgs::Float32 msg;
  msg.data = power;
  motor_power_pub_.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_power_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  motorPowerPublisher motor_power_publisher(nh, nhp);
  ros::spin();

  return 0;
}
