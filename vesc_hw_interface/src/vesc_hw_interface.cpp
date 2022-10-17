/*********************************************************************
 * Copyright (c) 2019, SoftBank Corp.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************/

#include "vesc_hw_interface/vesc_hw_interface.h"
#include <urdf_model/types.h>
#include <cmath>
#include "angles/angles.h"

namespace vesc_hw_interface
{
VescHwInterface::VescHwInterface()
  : vesc_interface_(std::string(), std::bind(&VescHwInterface::packetCallback, this, std::placeholders::_1),
                    std::bind(&VescHwInterface::errorCallback, this, std::placeholders::_1))
{
}

VescHwInterface::~VescHwInterface()
{
}

bool VescHwInterface::init(ros::NodeHandle& nh_root, ros::NodeHandle& nh)
{
  // reads a port name to open
  std::string port;
  if (!nh.getParam("port", port))
  {
    ROS_FATAL("VESC communication port parameter required.");
    ros::shutdown();
  }

  // attempts to open the serial port
  try
  {
    vesc_interface_.connect(port);
  }
  catch (serial::SerialException exception)
  {
    ROS_FATAL("Failed to connect to the VESC, %s.", exception.what());
    // ros::shutdown();
    return false;
  }

  // initializes the joint name
  nh.param<std::string>("joint_name", joint_name_, "joint_vesc");

  // loads joint limits
  std::string robot_description_name, robot_description;
  nh.param<std::string>("robot_description_name", robot_description_name, "/robot_description");

  // parses the urdf
  joint_type_ = "";
  if (nh.getParam(robot_description_name, robot_description))
  {
    const urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(robot_description);
    const urdf::JointConstSharedPtr urdf_joint = urdf->getJoint(joint_name_);

    if (getJointLimits(urdf_joint, joint_limits_))
    {
      ROS_INFO("Joint limits are loaded");
    }

    switch (urdf_joint->type)
    {
      case urdf::Joint::REVOLUTE:
        joint_type_ = "revolute";
        break;
      case urdf::Joint::CONTINUOUS:
        joint_type_ = "continuous";
        break;
      case urdf::Joint::PRISMATIC:
        joint_type_ = "prismatic";
        break;
    }
  }

  // initializes commands and states
  command_ = 0.0;
  position_ = 0.0;
  velocity_ = 0.0;
  effort_ = 0.0;
  rad_ = 0.0;

  // reads system parameters
  nh.param<double>("gear_ratio", gear_ratio_, 1.0);
  nh.param<double>("torque_const", torque_const_, 1.0);
  nh.param<int>("num_motor_pole_pairs", num_motor_pole_pairs_, 1);
  ROS_INFO("Gear ratio is set to %f", gear_ratio_);
  ROS_INFO("Torque constant is set to %f", torque_const_);
  ROS_INFO("The number of motor pole pairs is set to %d", num_motor_pole_pairs_);

  // reads driving mode setting
  // - assigns an empty string if param. is not found
  nh.param<std::string>("command_mode", command_mode_, "");
  ROS_INFO("command mode: %s", command_mode_.data());

  // registers a state handle and its interface
  hardware_interface::JointStateHandle state_handle(joint_name_, &position_, &velocity_, &effort_);
  joint_state_interface_.registerHandle(state_handle);
  registerInterface(&joint_state_interface_);

  // registers specified command handle and its interface
  if (command_mode_ == "position")
  {
    hardware_interface::JointHandle position_handle(joint_state_interface_.getHandle(joint_name_), &command_);
    joint_position_interface_.registerHandle(position_handle);
    registerInterface(&joint_position_interface_);

    joint_limits_interface::PositionJointSaturationHandle limit_handle(position_handle, joint_limits_);
    limit_position_interface_.registerHandle(limit_handle);

    // initializes the servo controller
    servo_controller_.init(nh, &vesc_interface_);
  }
  else if (command_mode_ == "velocity" || command_mode_ == "velocity_duty")
  {
    hardware_interface::JointHandle velocity_handle(joint_state_interface_.getHandle(joint_name_), &command_);
    joint_velocity_interface_.registerHandle(velocity_handle);
    registerInterface(&joint_velocity_interface_);

    joint_limits_interface::VelocityJointSaturationHandle limit_handle(velocity_handle, joint_limits_);
    limit_velocity_interface_.registerHandle(limit_handle);

    if (command_mode_ == "velocity_duty")
    {
      nh.param<double>("kp", kp_, 0.0);
      nh.param<double>("ki", ki_, 0.0);
      nh.param<double>("kd", kd_, 0.0);
      nh.param<double>("duty_multiplier", duty_multiplier_, 1.0);
      nh.param<double>("duty_limiter", duty_limiter_, 1.0);
      vesc_ready_ = false;
      displacement_ = 0.0;
      displacement_prev_ = 0.0;
      ROS_INFO("kp %f", kp_);
      ROS_INFO("ki %f", ki_);
      ROS_INFO("kd %f", kd_);
      ROS_INFO("duty_multiplier %f", duty_multiplier_);
      ROS_INFO("duty_limiter %f", duty_limiter_);
    }
  }
  else if (command_mode_ == "effort" || command_mode_ == "effort_duty")
  {
    hardware_interface::JointHandle effort_handle(joint_state_interface_.getHandle(joint_name_), &command_);
    joint_effort_interface_.registerHandle(effort_handle);
    registerInterface(&joint_effort_interface_);

    joint_limits_interface::EffortJointSaturationHandle limit_handle(effort_handle, joint_limits_);
    limit_effort_interface_.registerHandle(limit_handle);
  }
  else
  {
    ROS_ERROR("Verify your command mode setting");
    // ros::shutdown();
    return false;
  }

  // reads joint type setting
  nh.getParam("joint_type", joint_type_);
  ROS_INFO("joint type: %s", joint_type_.data());
  if ((joint_type_ != "revolute") && (joint_type_ != "continuous") && (joint_type_ != "prismatic"))
  {
    ROS_ERROR("Verify your joint type");
    return false;
  }
  return true;
}

void VescHwInterface::read()
{
  // requests joint states
  // function `packetCallback` will be called after receiving return packets
  vesc_interface_.requestState();
  return;
}

void VescHwInterface::read(const ros::Time& time, const ros::Duration& period)
{
  read();
  if (!vesc_ready_)
  {
    displacement_prev_ = displacement_;
  }
  double displacement_diff = displacement_ - displacement_prev_;
  if (fabs(displacement_diff) > num_motor_pole_pairs_ / 4)
  {
    displacement_diff = 0;
    vesc_ready_ = false;
  }
  if ((joint_type_ == "revolute") || (joint_type_ == "continuous"))
  {
    rad_ += displacement_diff / num_motor_pole_pairs_ * 2.0 * M_PI;
    position_ = angles::normalize_angle(rad_);
  }
  return;
}

void VescHwInterface::write()
{
  // sends commands
  if (command_mode_ == "position")
  {
    limit_position_interface_.enforceLimits(getPeriod());

    // executes PID control
    servo_controller_.control(command_, position_);
  }
  else if (command_mode_ == "velocity")
  {
    limit_velocity_interface_.enforceLimits(getPeriod());

    // converts the velocity unit: rad/s or m/s -> rpm -> erpm
    const double command_rpm = command_ * 60.0 / 2.0 / M_PI / gear_ratio_;
    const double command_erpm = command_rpm * static_cast<double>(num_motor_pole_pairs_);

    // sends a reference velocity command
    vesc_interface_.setSpeed(command_erpm);
  }
  else if (command_mode_ == "velocity_duty")
  {
    double target_vel_in = 0.0;
    target_vel_in = command_;
    double duty_out = 0.0;
    if (!vesc_ready_)
    {
      this->PIDControl(target_vel_in, &duty_out, true);
      vesc_ready_ = true;
    }
    else
    {
      this->PIDControl(target_vel_in, &duty_out, false);
    }
    vesc_interface_.setDutyCycle(duty_out);
  }
  else if (command_mode_ == "effort")
  {
    limit_effort_interface_.enforceLimits(getPeriod());

    // converts the command unit: Nm or N -> A
    const double command_current = command_ * gear_ratio_ / torque_const_;

    // sends a reference current command
    vesc_interface_.setCurrent(command_current);
  }
  else if (command_mode_ == "effort_duty")
  {
    command_ = std::max(-1.0, command_);
    command_ = std::min(1.0, command_);

    // sends a  duty command
    vesc_interface_.setDutyCycle(command_);
  }
  return;
}

int VescHwInterface::PIDControl(double target_vel, double* duty_out, bool init)
{
  static double p_tmp = 0.0, i_tmp = 0.0, i_prev = 0.0, d_tmp = 0.0;
  const double motor_hall_ppr = static_cast<double>(num_motor_pole_pairs_);
  const double i_duty_limit = 0.2;
  const double count_deviation_limit = static_cast<double>(num_motor_pole_pairs_);
  const double target_velocity_scaling_tmp = 1.0;  // 0.6

  double duty_limit = fabs(duty_limiter_);
  if (duty_limit > 1.0)
  {
    duty_limit = 1.0;
  }

  static long pose_sens = 0;
  pose_sens = static_cast<long>(displacement_);
  static int init_flag_tmp = 1;
  if (init)
  {
    init_flag_tmp = 1;
  }

  // initialize pid
  static double pose_target = 0;
  if (init_flag_tmp == 1)
  {
    pose_target = static_cast<double>(pose_sens);
    i_tmp = 0.0;
    p_tmp = 0.0;
    init_flag_tmp = 0;
    d_tmp = 0.0;
    this->CounterTD(pose_sens, true);
    *duty_out = 0.0;
  }
  else
  {
    pose_target += target_velocity_scaling_tmp * (target_vel * motor_hall_ppr / (2 * M_PI) / 50.0);
  }

  // cycle through
  if (pose_target > static_cast<double>(LONG_MAX))
  {
    pose_target += static_cast<double>(LONG_MIN);
  }
  else if (pose_target < static_cast<double>(LONG_MIN))
  {
    pose_target += static_cast<double>(LONG_MAX);
  }

  // pid controller
  // error limit
  if (static_cast<long>(pose_target) - static_cast<long>(pose_sens) > static_cast<long>(count_deviation_limit))
  {
    pose_target = static_cast<double>(pose_sens) + count_deviation_limit;
  }
  else if (static_cast<long>(pose_target) - static_cast<long>(pose_sens) < -static_cast<long>(count_deviation_limit))
  {
    pose_target = static_cast<double>(pose_sens) - count_deviation_limit;
  }

  double pose_target_diff = target_vel;
  double sens_target_diff = this->CounterTD(static_cast<long>(pose_sens), false) * 2.0 * M_PI / motor_hall_ppr * 50.0;

  d_tmp = pose_target_diff - sens_target_diff;
  p_tmp = static_cast<double>(static_cast<long>(pose_target) - static_cast<long>(pose_sens));
  i_prev = i_tmp;
  i_tmp += (p_tmp / 50.0);
  double duty_out_tmp;
  duty_out_tmp = (kp_ * p_tmp + ki_ * i_tmp + kd_ * d_tmp);

  if (duty_out_tmp > duty_limit)
  {
    duty_out_tmp = duty_limit;
    if (i_tmp > i_prev)
    {  // anti reset wind up
      i_tmp = i_prev;
      duty_out_tmp = (kp_ * p_tmp + ki_ * i_tmp + kd_ * d_tmp);
    }
  }
  else if (duty_out_tmp < -duty_limit)
  {
    duty_out_tmp = -duty_limit;
    if (i_tmp < i_prev)
    {  // anti reset wind up
      i_tmp = i_prev;
      duty_out_tmp = (kp_ * p_tmp + ki_ * i_tmp + kd_ * d_tmp);
    }
  }
  if (ki_ * i_tmp > i_duty_limit)
  {  // anti reset wind up
    i_tmp = i_duty_limit / ki_;
  }
  else if (ki_ * i_tmp < -i_duty_limit)
  {
    i_tmp = -i_duty_limit / ki_;
  }

  if (init_flag_tmp == 0)
  {
    *duty_out = duty_multiplier_ * duty_out_tmp;
  }
  if (*duty_out > 1.0)
  {
    *duty_out = 1.0;
  }
  else if (*duty_out < -1.0)
  {
    *duty_out = -1.0;
  }

  // torque off when stop
  static int duty_zero_counter = 0;
  static bool stop_flag = false;
  stop_flag = fabs(command_) < 0.0001;
  if (!stop_flag)
  {
    duty_zero_counter = 0;
  }
  else if (duty_zero_counter < 10)
  {
    duty_zero_counter++;
  }
  if (duty_zero_counter == 10)
  {
    *duty_out = 0.0;
    vesc_ready_ = false;
  }
  return 0;
}

double VescHwInterface::CounterTD(long count_in, bool init)
{
  static uint16_t counter_changed_log[10][2] = {};
  static double counter_td_tmp[10] = {};
  static uint16_t counter_changed_single = 1;
  int i = 0;
  double output = 0.0;
  if (init)
  {
    counter_changed_single = 1;
    for (i = 0; i < 10; i++)
    {
      counter_changed_log[i][0] = static_cast<uint16_t>(count_in);
      counter_changed_log[i][1] = 100;
      counter_td_tmp[i] = 0;
    }
    return 0.0;
  }
  if (counter_changed_log[0][0] != static_cast<uint16_t>(count_in))
  {
    for (i = 1; i < 10; i++)
    {
      counter_changed_log[10 - i][0] = counter_changed_log[9 - i][0];
    }
    counter_changed_log[0][0] = static_cast<uint16_t>(count_in);
    counter_changed_log[0][1] = counter_changed_single;
    counter_changed_single = 1;
  }
  else
  {
    if (counter_changed_single > counter_changed_log[0][1])
    {
      counter_changed_log[0][1] = counter_changed_single;
    }
    if (counter_changed_single < 100)
    {
      counter_changed_single++;
    }
  }
  for (i = 1; i < 10; i++)
  {
    counter_td_tmp[10 - i] = counter_td_tmp[9 - i];
  }
  counter_td_tmp[0] = static_cast<double>(counter_changed_log[0][0] - counter_changed_log[1][0]) /
                      static_cast<double>(counter_changed_log[0][1]);
  output = counter_td_tmp[0];
  if (fabs(output) > 100.0)
  {  // 変化量が異常だった場合0にする(エラー処理)
    output = 0.0;
  }
  return output;
}

void VescHwInterface::write(const ros::Time& time, const ros::Duration& period)
{
  write();
  return;
}

ros::Time VescHwInterface::getTime() const
{
  return ros::Time::now();
}

ros::Duration VescHwInterface::getPeriod() const
{
  return ros::Duration(0.01);
}

void VescHwInterface::packetCallback(const std::shared_ptr<VescPacket const>& packet)
{
  if (packet->getName() == "Values")
  {
    std::shared_ptr<VescPacketValues const> values = std::dynamic_pointer_cast<VescPacketValues const>(packet);

    const double current = values->getMotorCurrent();
    const double velocity_rpm = values->getVelocityERPM() / static_cast<double>(num_motor_pole_pairs_);
    const double position_pulse = values->getPosition();

    // 3.0 represents the number of hall sensors
    if (joint_type_ == "prismatic")
    {
      position_ = position_pulse / num_motor_pole_pairs_ / 3.0 * gear_ratio_ -
                  servo_controller_.getZeroPosition();  // unit: rad or m
    }
    velocity_ = velocity_rpm / 60.0 * 2.0 * M_PI * gear_ratio_;  // unit: rad/s or m/s
    effort_ = current * torque_const_ / gear_ratio_;             // unit: Nm or N
    displacement_prev_ = displacement_;
    displacement_ = values->getPosition();
  }

  return;
}

void VescHwInterface::errorCallback(const std::string& error)
{
  ROS_ERROR("%s", error.c_str());
  return;
}

}  // namespace vesc_hw_interface

PLUGINLIB_EXPORT_CLASS(vesc_hw_interface::VescHwInterface, hardware_interface::RobotHW)
