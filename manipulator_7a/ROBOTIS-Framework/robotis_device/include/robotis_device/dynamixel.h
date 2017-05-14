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
 * dynamixel.h
 *
 *  Created on: 2015. 12. 8.
 *      Author: zerom
 */

#ifndef ROBOTIS_DEVICE_DYNAMIXEL_H_
#define ROBOTIS_DEVICE_DYNAMIXEL_H_


#include <map>
#include <vector>
#include <string>

#include "control_table_item.h"
#include "device.h"
#include "dynamixel_state.h"

namespace robotis_framework
{

class Dynamixel : public Device
{
public:
  std::string     ctrl_module_name_;
  DynamixelState *dxl_state_;

  double  velocity_to_value_ratio_;
  double  torque_to_current_value_ratio_;

  int32_t value_of_0_radian_position_;
  int32_t value_of_min_radian_position_;
  int32_t value_of_max_radian_position_;
  double  min_radian_;
  double  max_radian_;

  ControlTableItem *torque_enable_item_;
  ControlTableItem *present_position_item_;
  ControlTableItem *present_velocity_item_;
  ControlTableItem *present_current_item_;
  ControlTableItem *goal_position_item_;
  ControlTableItem *goal_velocity_item_;
  ControlTableItem *goal_current_item_;
  ControlTableItem *position_p_gain_item_;
  ControlTableItem *position_i_gain_item_;
  ControlTableItem *position_d_gain_item_;
  ControlTableItem *velocity_p_gain_item_;
  ControlTableItem *velocity_i_gain_item_;
  ControlTableItem *velocity_d_gain_item_;

  Dynamixel(int id, std::string model_name, float protocol_version);

  double  convertValue2Radian(int32_t value);
  int32_t convertRadian2Value(double radian);

  double  convertValue2Velocity(int32_t value);
  int32_t convertVelocity2Value(double velocity);

  double  convertValue2Torque(int16_t value);
  int16_t convertTorque2Value(double torque);
};

}


#endif /* ROBOTIS_DEVICE_DYNAMIXEL_H_ */
