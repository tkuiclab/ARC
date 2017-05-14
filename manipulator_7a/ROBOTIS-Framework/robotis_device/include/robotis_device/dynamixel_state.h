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
 * dynamixel_state.h
 *
 *  Created on: 2015. 12. 8.
 *      Author: zerom
 */

#ifndef ROBOTIS_DEVICE_DYNAMIXEL_STATE_H_
#define ROBOTIS_DEVICE_DYNAMIXEL_STATE_H_

#include <stdint.h>

#include "time_stamp.h"

#define INDIRECT_DATA_1     "indirect_data_1"
#define INDIRECT_ADDRESS_1  "indirect_address_1"

namespace robotis_framework
{

class DynamixelState
{
public:
  TimeStamp update_time_stamp_;

  double    present_position_;
  double    present_velocity_;
  double    present_torque_;
  double    goal_position_;
  double    goal_velocity_;
  double    goal_torque_;
  int       position_p_gain_;
  int       position_i_gain_;
  int       position_d_gain_;
  int       velocity_p_gain_;
  int       velocity_i_gain_;
  int       velocity_d_gain_;

  std::map<std::string, uint32_t> bulk_read_table_;

  double    position_offset_;

  DynamixelState()
    : update_time_stamp_(0, 0),
      present_position_(0.0),
      present_velocity_(0.0),
      present_torque_(0.0),
      goal_position_(0.0),
      goal_velocity_(0.0),
      goal_torque_(0.0),
      position_p_gain_(0),
      position_i_gain_(0),
      position_d_gain_(0),
      velocity_p_gain_(0),
      velocity_i_gain_(0),
      velocity_d_gain_(0),
      position_offset_(0)
  {
    bulk_read_table_.clear();
  }
};

}


#endif /* ROBOTIS_DEVICE_DYNAMIXEL_STATE_H_ */
