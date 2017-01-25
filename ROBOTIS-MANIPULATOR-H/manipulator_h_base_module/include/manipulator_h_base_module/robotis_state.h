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

#ifndef MANIPULATOR_BASE_MODULE_ROBOTIS_STATE_H_
#define MANIPULATOR_BASE_MODULE_ROBOTIS_STATE_H_

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include "robotis_math/robotis_math.h"

#include "manipulator_h_base_module_msgs/JointPose.h"
#include "manipulator_h_base_module_msgs/KinematicsPose.h"
#include "manipulator_h_kinematics_dynamics/manipulator_h_kinematics_dynamics.h"

namespace robotis_manipulator_h
{

class RobotisState
{
public:
  RobotisState();
  ~RobotisState();

  bool    is_moving_;

  // trajectory
  int     cnt_;
  int     all_time_steps_;
  double  mov_time_;
  double  smp_time_;

  Eigen::MatrixXd calc_joint_tra_;
  Eigen::MatrixXd calc_task_tra_;

  Eigen::MatrixXd joint_ini_pose_;

  // msgs
  manipulator_h_base_module_msgs::JointPose joint_pose_msg_;
  manipulator_h_base_module_msgs::KinematicsPose kinematics_pose_msg_;

  // inverse kinematics
  bool ik_solve_;
  Eigen::MatrixXd ik_target_position_;
  Eigen::MatrixXd ik_start_rotation_, ik_target_rotation_;
  int ik_id_start_, ik_id_end_;

  void setInverseKinematics(int cnt, Eigen::MatrixXd start_rotation);
};

}

#endif /* MANIPULATOR_BASE_MODULE_ROBOTIS_STATE_H_ */
