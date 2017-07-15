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
 * Link.cpp
 *
 *  Created on: Jan 11, 2016
 *      Author: sch
 */

#include "manipulator_h_base_module/robotis_state.h"

using namespace robotis_manipulator_h;

RobotisState::RobotisState()
{
    is_moving_ = false;

    cnt_      = 0;
    mov_time_ = 1.0;
    smp_time_ = 0.008;
    all_time_steps_ = int(mov_time_ / smp_time_) + 1;

    calc_joint_tra_ = Eigen::MatrixXd::Zero(all_time_steps_, MAX_JOINT_ID + 1);
    calc_task_tra_  = Eigen::MatrixXd::Zero(all_time_steps_, 3);
    calc_fai_tra    = Eigen::MatrixXd::Zero(all_time_steps_, 1);

    joint_ini_pose_ = Eigen::MatrixXd::Zero(MAX_JOINT_ID + 1, 1);

    // for inverse kinematics;
    ik_solve_ = false;

    ik_target_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);

    ik_start_rotation_  = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    ik_target_rotation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    line_ik_pos         = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
    line_ik_rpy         = Eigen::MatrixXd::Zero(1, 3);

    ik_target_fai = 0;
    ik_cmd_fai    = 0;

    ik_id_start_ = 0;
    ik_id_end_   = 0;
}

RobotisState::~RobotisState()
{
}

/* ----- Setting pos, ori, fai of next tick for ik ----- */
void RobotisState::setInverseKinematics()
{
    for (int dim = 0; dim < 3; dim++)
        ik_target_position_.coeffRef(dim, 0) = calc_task_tra_.coeff(cnt_, dim);

    Eigen::Quaterniond start_quaternion = robotis_framework::convertRotationToQuaternion(ik_start_rotation_);
    
    // convert error rot matrix to rpy and then fix it
    Eigen::MatrixXd tmp_start_pry = robotis_framework::convertQuaternionToRPY(start_quaternion);
    // tmp_start_pry(2) += 90*M_PI/180.0;
    std::cout<< "tmp_start_pry = " <<tmp_start_pry <<"\n";

    // convert the correct rpy to quaternion
    start_quaternion = robotis_framework::convertEulerToQuaternion( tmp_start_pry(1),
                                                                    tmp_start_pry(0),
                                                                    tmp_start_pry(2));//OK

    // Calculate Target quaternion
    Eigen::Quaterniond target_quaternion(kinematics_pose_msg_.pose.orientation.w,
                                         kinematics_pose_msg_.pose.orientation.x,
                                         kinematics_pose_msg_.pose.orientation.y,
                                         kinematics_pose_msg_.pose.orientation.z);

    double count = (double)cnt_ / (double)all_time_steps_;

    Eigen::Quaterniond quaternion = start_quaternion.slerp(count, target_quaternion);
    // ik_target_rotation_ = robotis_framework::convertQuaternionToRotation(quaternion);
    ik_target_rotation_ = robotis_framework::convertQuat2Rotation(quaternion);


    // std::cout << "start_quaternion \n" 
    //           << start_quaternion.w() <<", " 
    //           << start_quaternion.x() <<", "
    //           << start_quaternion.y() <<", "
    //           << start_quaternion.z() <<", "<< std::endl;

    std::cout << "target_quaternion \n" 
              << target_quaternion.w() <<", " 
              << target_quaternion.x() <<", "
              << target_quaternion.y() <<", "
              << target_quaternion.z() <<", "<< std::endl;

    // std::cout << "slerp_quaternion \n" 
    //           << quaternion.w() <<", " 
    //           << quaternion.x() <<", "
    //           << quaternion.y() <<", "
    //           << quaternion.z() <<", "<< std::endl;
    // std::cout << "target_quaternion \n" << target_quaternion << std::endl;
    // std::cout << "setInverseKinematics start: \n" << ik_start_rotation_  << std::endl;
    // std::cout << "setInverseKinematics target:\n" << ik_target_rotation_ << std::endl;

    /* get next step euler */
    line_ik_rpy = robotis_framework::convertRotationToRPY(ik_target_rotation_);
    double tmp = line_ik_rpy(0,0);
    line_ik_rpy(0,0) = line_ik_rpy(1,0);
    line_ik_rpy(1,0) = tmp;
    std::cout<<"in set ik:line_ik_rpy_2 =  " <<line_ik_rpy <<"\n";

    /* get next step redundancy */
    ik_target_fai = calc_fai_tra(cnt_, 0);
}
