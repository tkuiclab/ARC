/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*
 * dynamixel.cpp
 *
 *  Created on: 2015. 12. 8.
 *      Author: zerom
 */

#include "robotis_device/dynamixel.h"

using namespace robotis_framework;

Dynamixel::Dynamixel(int id, std::string model_name, float protocol_version)
  : ctrl_module_name_("none"),
    torque_to_current_value_ratio_(1.0),
    velocity_to_value_ratio_(1.0),
    value_of_0_radian_position_(0),
    value_of_min_radian_position_(0),
    value_of_max_radian_position_(0),
    min_radian_(-3.14159265),
    max_radian_(3.14159265),
    torque_enable_item_(0),
    present_position_item_(0),
    present_velocity_item_(0),
    present_current_item_(0),
    goal_position_item_(0),
    goal_velocity_item_(0),
    goal_current_item_(0),
    position_p_gain_item_(0),
    position_i_gain_item_(0),
    position_d_gain_item_(0),
    velocity_p_gain_item_(0),
    velocity_i_gain_item_(0),
    velocity_d_gain_item_(0)
{
  this->id_ = id;
  this->model_name_ = model_name;
  this->port_name_ = "";
  this->protocol_version_ = protocol_version;

  ctrl_table_.clear();
  dxl_state_ = new DynamixelState();

  bulk_read_items_.clear();
}

double Dynamixel::convertValue2Radian(int32_t value)
{
  double radian = 0.0;
  if (value > value_of_0_radian_position_)
  {
    if (max_radian_ <= 0)
      return max_radian_;

    radian = (double) (value - value_of_0_radian_position_) * max_radian_
               / (double) (value_of_max_radian_position_ - value_of_0_radian_position_);
  }
  else if (value < value_of_0_radian_position_)
  {
    if (min_radian_ >= 0)
      return min_radian_;

    radian = (double) (value - value_of_0_radian_position_) * min_radian_
               / (double) (value_of_min_radian_position_ - value_of_0_radian_position_);
  }

  if (radian > max_radian_)
    return max_radian_;
  else if (radian < min_radian_)
    return min_radian_;

  return radian;
}

int32_t Dynamixel::convertRadian2Value(double radian)
{
  int32_t value = 0;
  if (radian > 0)
  {
    if (value_of_max_radian_position_ <= value_of_0_radian_position_)
      return value_of_max_radian_position_;

    value = (radian * (value_of_max_radian_position_ - value_of_0_radian_position_) / max_radian_)
                + value_of_0_radian_position_;
  }
  else if (radian < 0)
  {
    if (value_of_min_radian_position_ >= value_of_0_radian_position_)
      return value_of_min_radian_position_;

    value = (radian * (value_of_min_radian_position_ - value_of_0_radian_position_) / min_radian_)
                + value_of_0_radian_position_;
  }
  else
    value = value_of_0_radian_position_;

  if (value > value_of_max_radian_position_)
    return value_of_max_radian_position_;
  else if (value < value_of_min_radian_position_)
    return value_of_min_radian_position_;

  return value;
}

double Dynamixel::convertValue2Velocity(int32_t value)
{
  return (double) value / velocity_to_value_ratio_;
}

int32_t Dynamixel::convertVelocity2Value(double velocity)
{
  return (int32_t) (velocity * velocity_to_value_ratio_);;
}

double Dynamixel::convertValue2Torque(int16_t value)
{
  return (double) value / torque_to_current_value_ratio_;
}

int16_t Dynamixel::convertTorque2Value(double torque)
{
  return (int16_t) (torque * torque_to_current_value_ratio_);
}
