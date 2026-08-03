#include <aerial_robot_simulation/mujoco/thrust_visualizer_plugin.h>

#include <algorithm>
#include <cmath>

#if MJR_ROS_VERSION == ROS_1
#include <pluginlib/class_list_macros.h>
#else
#include <pluginlib/class_list_macros.hpp>
#endif

namespace mujoco_ros
{

namespace
{
constexpr char kLogName[] = "thrust_visualizer";
}

bool ThrustVisualizerPlugin::Load(const mjModel* model, mjData* /*data*/)
{
  applyVisualDefaults(model);
  if (!configureFromParams())
  {
    return false;
  }

  arrows_.clear();
  const bool success = actuator_names_.empty() ? buildAutomaticArrowSources(model) : buildConfiguredArrowSources(model);
  if (!success || arrows_.empty())
  {
#if MJR_ROS_VERSION == ROS_1
    ROS_ERROR_NAMED("thrust_visualizer", "No actuator-site pairs were found for thrust visualization.");
#else
    RCLCPP_ERROR(get_node()->get_logger(), "No actuator-site pairs were found for thrust visualization.");
#endif
    return false;
  }

#if MJR_ROS_VERSION == ROS_1
  ROS_INFO_STREAM_NAMED("thrust_visualizer", "Loaded thrust visualizer with " << arrows_.size() << " arrow(s), scale="
                                                                              << scale_ << ", width=" << width_ << ".");
#else
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "Loaded thrust visualizer with " << arrows_.size() << " arrow(s), scale="
                                                                                 << scale_ << ", width=" << width_ << ".");
#endif
  return true;
}

void ThrustVisualizerPlugin::Reset()
{
}

void ThrustVisualizerPlugin::RenderCallback(const mjModel* model, mjData* data, mjvScene* scene)
{
  for (const auto& arrow : arrows_)
  {
    if (scene->ngeom >= scene->maxgeom)
    {
      mj_warning(data, mjWARN_VGEOMFULL, scene->maxgeom);
      return;
    }

    appendArrow(model, data, scene, arrow);
  }
}

void ThrustVisualizerPlugin::applyVisualDefaults(const mjModel* model)
{
  scale_ = model->vis.map.force / std::max(static_cast<mjtNum>(mjMINVAL), model->stat.meanmass);
  width_ = model->stat.meansize * model->vis.scale.forcewidth;
  if (width_ <= 0.0)
  {
    width_ = model->vis.scale.forcewidth;
  }

  for (int i = 0; i < 4; ++i)
  {
    rgba_[i] = model->vis.rgba.contactforce[i];
  }
}

bool ThrustVisualizerPlugin::configureFromParams()
{
#if MJR_ROS_VERSION == ROS_1
  readDoubleParam(rosparam_config_, "scale", scale_);
  readDoubleParam(rosparam_config_, "width", width_);
  readDoubleParam(rosparam_config_, "min_force", min_force_);
  readDoubleParam(rosparam_config_, "max_length", max_length_);
  readBoolParam(rosparam_config_, "use_abs_force", use_abs_force_);
  readBoolParam(rosparam_config_, "direction_from_gear", direction_from_gear_);
  readBoolParam(rosparam_config_, "include_all_site_actuators", include_all_site_actuators_);
  readRgbaParam(rosparam_config_, "rgba", rgba_);

  actuator_names_ = readStringArrayParam(rosparam_config_, "actuator_names");
  site_names_ = readStringArrayParam(rosparam_config_, "site_names");
#else
  readDoubleParam("scale", scale_);
  readDoubleParam("width", width_);
  readDoubleParam("min_force", min_force_);
  readDoubleParam("max_length", max_length_);
  readBoolParam("use_abs_force", use_abs_force_);
  readBoolParam("direction_from_gear", direction_from_gear_);
  readBoolParam("include_all_site_actuators", include_all_site_actuators_);
  readRgbaParam("rgba", rgba_);

  actuator_names_ = readStringArrayParam("actuator_names");
  site_names_ = readStringArrayParam("site_names");
#endif

  if (!site_names_.empty() && site_names_.size() != actuator_names_.size())
  {
#if MJR_ROS_VERSION == ROS_1
    ROS_ERROR_NAMED("thrust_visualizer", "`site_names` must be omitted or have the same length as `actuator_names`.");
#else
    RCLCPP_ERROR(get_node()->get_logger(),
                 "`site_names` must be omitted or have the same length as `actuator_names`.");
#endif
    return false;
  }

  if (scale_ <= 0.0)
  {
#if MJR_ROS_VERSION == ROS_1
    ROS_ERROR_NAMED("thrust_visualizer", "`scale` must be positive.");
#else
    RCLCPP_ERROR(get_node()->get_logger(), "`scale` must be positive.");
#endif
    return false;
  }

  if (width_ <= 0.0)
  {
#if MJR_ROS_VERSION == ROS_1
    ROS_ERROR_NAMED("thrust_visualizer", "`width` must be positive.");
#else
    RCLCPP_ERROR(get_node()->get_logger(), "`width` must be positive.");
#endif
    return false;
  }

  min_force_ = std::max(0.0, min_force_);
  max_length_ = std::max(0.0, max_length_);

  return true;
}

bool ThrustVisualizerPlugin::buildConfiguredArrowSources(const mjModel* model)
{
  bool success = true;
  for (std::size_t i = 0; i < actuator_names_.size(); ++i)
  {
    const std::string& actuator_name = actuator_names_[i];
    const std::string site_name = site_names_.empty() ? std::string() : site_names_[i];
    const int actuator_id = mj_name2id(const_cast<mjModel*>(model), mjOBJ_ACTUATOR, actuator_name.c_str());
    if (actuator_id < 0)
    {
#if MJR_ROS_VERSION == ROS_1
      ROS_ERROR_STREAM_NAMED("thrust_visualizer", "Actuator `" << actuator_name << "` was not found.");
#else
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Actuator `" << actuator_name << "` was not found.");
#endif
      success = false;
      continue;
    }

    const int site_id = findSiteForActuator(model, actuator_id, site_name);
    if (site_id < 0)
    {
#if MJR_ROS_VERSION == ROS_1
      ROS_ERROR_STREAM_NAMED("thrust_visualizer", "No site was found for actuator `" << actuator_name << "`.");
#else
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "No site was found for actuator `" << actuator_name << "`.");
#endif
      success = false;
      continue;
    }

    const char* resolved_site_name = mj_id2name(const_cast<mjModel*>(model), mjOBJ_SITE, site_id);
    arrows_.push_back({ actuator_id, site_id, actuator_name, resolved_site_name ? resolved_site_name : "" });
  }

  return success;
}

bool ThrustVisualizerPlugin::buildAutomaticArrowSources(const mjModel* model)
{
  if (!include_all_site_actuators_)
  {
#if MJR_ROS_VERSION == ROS_1
    ROS_ERROR_NAMED("thrust_visualizer", "`actuator_names` is empty and `include_all_site_actuators` is false.");
#else
    RCLCPP_ERROR(get_node()->get_logger(),
                 "`actuator_names` is empty and `include_all_site_actuators` is false.");
#endif
    return false;
  }

  for (int actuator_id = 0; actuator_id < model->nu; ++actuator_id)
  {
    if (model->actuator_trntype[actuator_id] != mjTRN_SITE)
    {
      continue;
    }

    const int site_id = model->actuator_trnid[2 * actuator_id];
    if (site_id < 0 || site_id >= model->nsite)
    {
      continue;
    }

    const char* actuator_name = mj_id2name(const_cast<mjModel*>(model), mjOBJ_ACTUATOR, actuator_id);
    const char* site_name = mj_id2name(const_cast<mjModel*>(model), mjOBJ_SITE, site_id);
    arrows_.push_back({ actuator_id, site_id, actuator_name ? actuator_name : "", site_name ? site_name : "" });
  }

  return !arrows_.empty();
}

int ThrustVisualizerPlugin::findSiteForActuator(const mjModel* model, int actuator_id,
                                                const std::string& fallback_site_name) const
{
  if (!fallback_site_name.empty())
  {
    return mj_name2id(const_cast<mjModel*>(model), mjOBJ_SITE, fallback_site_name.c_str());
  }

  if (model->actuator_trntype[actuator_id] == mjTRN_SITE)
  {
    return model->actuator_trnid[2 * actuator_id];
  }

  const char* actuator_name = mj_id2name(const_cast<mjModel*>(model), mjOBJ_ACTUATOR, actuator_id);
  if (!actuator_name)
  {
    return -1;
  }

  return mj_name2id(const_cast<mjModel*>(model), mjOBJ_SITE, actuator_name);
}

void ThrustVisualizerPlugin::appendArrow(const mjModel* model, mjData* data, mjvScene* scene,
                                         const ArrowSource& arrow) const
{
  double force = data->ctrl[arrow.actuator_id];
  if (use_abs_force_)
  {
    force = std::abs(force);
  }

  if (std::abs(force) < min_force_)
  {
    return;
  }

  mjtNum length = static_cast<mjtNum>(force * scale_);
  if (max_length_ > 0.0)
  {
    length = static_cast<mjtNum>(std::clamp(static_cast<double>(length), -max_length_, max_length_));
  }

  mjtNum local_dir[3] = { 0.0, 0.0, 1.0 };
  if (direction_from_gear_)
  {
    const mjtNum* gear = model->actuator_gear + 6 * arrow.actuator_id;
    if (mju_norm3(gear) > 1.0e-9)
    {
      mju_copy3(local_dir, gear);
      mju_normalize3(local_dir);
    }
  }

  const mjtNum* site_pos = data->site_xpos + 3 * arrow.site_id;
  const mjtNum* site_mat = data->site_xmat + 9 * arrow.site_id;
  mjtNum world_dir[3];
  mju_mulMatVec3(world_dir, site_mat, local_dir);

  mjtNum tip[3] = { site_pos[0] + length * world_dir[0], site_pos[1] + length * world_dir[1],
                    site_pos[2] + length * world_dir[2] };

  mjvGeom* geom = scene->geoms + scene->ngeom++;
  mjv_initGeom(geom, mjGEOM_ARROW, nullptr, nullptr, nullptr, rgba_.data());
  mjv_connector(geom, mjGEOM_ARROW, static_cast<mjtNum>(width_), site_pos, tip);
}

#if MJR_ROS_VERSION == ROS_1
bool ThrustVisualizerPlugin::readDoubleParam(const XmlRpc::XmlRpcValue& config, const std::string& name, double& value)
{
  if (!config.hasMember(name))
  {
    return false;
  }

  value = xmlRpcNumberToDouble(config[name], value);
  return true;
}

bool ThrustVisualizerPlugin::readBoolParam(const XmlRpc::XmlRpcValue& config, const std::string& name, bool& value)
{
  if (!config.hasMember(name) || config[name].getType() != XmlRpc::XmlRpcValue::TypeBoolean)
  {
    return false;
  }

  value = static_cast<bool>(config[name]);
  return true;
}

std::vector<std::string> ThrustVisualizerPlugin::readStringArrayParam(const XmlRpc::XmlRpcValue& config,
                                                                      const std::string& name)
{
  std::vector<std::string> values;
  if (!config.hasMember(name))
  {
    return values;
  }

  const XmlRpc::XmlRpcValue& xml_values = config[name];
  if (xml_values.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_WARN_STREAM_NAMED(kLogName, "`" << name << "` must be a string array. Ignoring it.");
    return values;
  }

  for (int i = 0; i < xml_values.size(); ++i)
  {
    if (xml_values[i].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_WARN_STREAM_NAMED(kLogName, "`" << name << "` contains a non-string value. Ignoring it.");
      values.clear();
      return values;
    }
    values.push_back(static_cast<std::string>(xml_values[i]));
  }

  return values;
}

bool ThrustVisualizerPlugin::readRgbaParam(const XmlRpc::XmlRpcValue& config, const std::string& name,
                                           std::array<float, 4>& value)
{
  if (!config.hasMember(name))
  {
    return false;
  }

  const XmlRpc::XmlRpcValue& rgba = config[name];
  if (rgba.getType() != XmlRpc::XmlRpcValue::TypeArray || rgba.size() != 4)
  {
    ROS_WARN_STREAM_NAMED(kLogName, "`" << name << "` must be an array with 4 numbers. Ignoring it.");
    return false;
  }

  for (int i = 0; i < 4; ++i)
  {
    value[i] = static_cast<float>(xmlRpcNumberToDouble(rgba[i], value[i]));
  }

  return true;
}

double ThrustVisualizerPlugin::xmlRpcNumberToDouble(const XmlRpc::XmlRpcValue& value, double default_value)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    return static_cast<double>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    return static_cast<int>(value);
  }
  return default_value;
}
#else
bool ThrustVisualizerPlugin::readDoubleParam(const std::string& name, double& value) const
{
  if (!get_node()->has_parameter(name))
  {
    return false;
  }

  const auto parameter = get_node()->get_parameter(name);
  switch (parameter.get_type())
  {
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      value = parameter.as_double();
      return true;
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      value = static_cast<double>(parameter.as_int());
      return true;
    default:
      RCLCPP_WARN_STREAM(get_node()->get_logger(), "`" << name << "` must be numeric. Ignoring it.");
      return false;
  }
}

bool ThrustVisualizerPlugin::readBoolParam(const std::string& name, bool& value) const
{
  if (!get_node()->has_parameter(name))
  {
    return false;
  }

  const auto parameter = get_node()->get_parameter(name);
  if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
  {
    RCLCPP_WARN_STREAM(get_node()->get_logger(), "`" << name << "` must be boolean. Ignoring it.");
    return false;
  }

  value = parameter.as_bool();
  return true;
}

std::vector<std::string> ThrustVisualizerPlugin::readStringArrayParam(const std::string& name) const
{
  std::vector<std::string> values;
  if (!get_node()->has_parameter(name))
  {
    return values;
  }

  const auto parameter = get_node()->get_parameter(name);
  if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
  {
    RCLCPP_WARN_STREAM(get_node()->get_logger(), "`" << name << "` must be a string array. Ignoring it.");
    return values;
  }

  return parameter.as_string_array();
}

bool ThrustVisualizerPlugin::readRgbaParam(const std::string& name, std::array<float, 4>& value) const
{
  if (!get_node()->has_parameter(name))
  {
    return false;
  }

  const auto parameter = get_node()->get_parameter(name);
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
  {
    const auto rgba = parameter.as_double_array();
    if (rgba.size() != 4)
    {
      RCLCPP_WARN_STREAM(get_node()->get_logger(), "`" << name << "` must have 4 numbers. Ignoring it.");
      return false;
    }

    for (int i = 0; i < 4; ++i)
    {
      value[i] = static_cast<float>(rgba[i]);
    }
    return true;
  }

  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
  {
    const auto rgba = parameter.as_integer_array();
    if (rgba.size() != 4)
    {
      RCLCPP_WARN_STREAM(get_node()->get_logger(), "`" << name << "` must have 4 numbers. Ignoring it.");
      return false;
    }

    for (int i = 0; i < 4; ++i)
    {
      value[i] = static_cast<float>(rgba[i]);
    }
    return true;
  }

  RCLCPP_WARN_STREAM(get_node()->get_logger(), "`" << name << "` must be a numeric array. Ignoring it.");
  return false;
}
#endif

}  // namespace mujoco_ros

PLUGINLIB_EXPORT_CLASS(mujoco_ros::ThrustVisualizerPlugin, mujoco_ros::MujocoPlugin)
