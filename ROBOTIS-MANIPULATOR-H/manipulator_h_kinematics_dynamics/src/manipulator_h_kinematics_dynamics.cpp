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

#include <iostream>
#include "manipulator_h_kinematics_dynamics/manipulator_h_kinematics_dynamics.h"

namespace robotis_manipulator_h
{

ManipulatorKinematicsDynamics::ManipulatorKinematicsDynamics()
{
}
ManipulatorKinematicsDynamics::~ManipulatorKinematicsDynamics()
{
}

ManipulatorKinematicsDynamics::ManipulatorKinematicsDynamics(TreeSelect tree)
{
  for (int id = 0; id <= ALL_JOINT_ID; id++)
    manipulator_link_data_[id] = new LinkData();

  if (tree == ARM)
  {
    manipulator_link_data_[0]->name_    = "base";
    manipulator_link_data_[0]->parent_  = -1;
    manipulator_link_data_[0]->sibling_ = -1;
    manipulator_link_data_[0]->child_   = 1;
    manipulator_link_data_[0]->mass_    = 0.0;
    manipulator_link_data_[0]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[0]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[0]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[0]->joint_limit_max_   =  100.0;
    manipulator_link_data_[0]->joint_limit_min_   = -100.0;
    manipulator_link_data_[0]->inertia_           = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    manipulator_link_data_[1]->name_    = "joint1";
    manipulator_link_data_[1]->parent_  = 0;
    manipulator_link_data_[1]->sibling_ = -1;
    manipulator_link_data_[1]->child_   = 2;
    manipulator_link_data_[1]->mass_    = 0.85644;
    manipulator_link_data_[1]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.126);
    manipulator_link_data_[1]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0, 0.0, 1.0);
    manipulator_link_data_[1]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[1]->joint_limit_max_   =  0.9 * M_PI;
    manipulator_link_data_[1]->joint_limit_min_   = -0.9 * M_PI;
    manipulator_link_data_[1]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

    manipulator_link_data_[2]->name_    = "joint2";
    manipulator_link_data_[2]->parent_  = 1;
    manipulator_link_data_[2]->sibling_ = -1;
    manipulator_link_data_[2]->child_   = 3;
    manipulator_link_data_[2]->mass_    = 0.94658;
    manipulator_link_data_[2]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.06900, 0.033);
    manipulator_link_data_[2]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    manipulator_link_data_[2]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[2]->joint_limit_max_   =  0.5 * M_PI;
    manipulator_link_data_[2]->joint_limit_min_   = -0.5 * M_PI;
    manipulator_link_data_[2]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

    manipulator_link_data_[3]->name_    = "joint3";
    manipulator_link_data_[3]->parent_  = 2;
    manipulator_link_data_[3]->sibling_ = -1;
    manipulator_link_data_[3]->child_   = 4;
    manipulator_link_data_[3]->mass_    = 1.30260;
    manipulator_link_data_[3]->relative_position_ = robotis_framework::getTransitionXYZ(0.03000, -0.01150, 0.26400);
    manipulator_link_data_[3]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    manipulator_link_data_[3]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[3]->joint_limit_max_   =  0.9 * M_PI;
    manipulator_link_data_[3]->joint_limit_min_   = -0.9 * M_PI;
    manipulator_link_data_[3]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

    manipulator_link_data_[4]->name_    = "joint4";
    manipulator_link_data_[4]->parent_  = 3;
    manipulator_link_data_[4]->sibling_ = -1;
    manipulator_link_data_[4]->child_   = 5;
    manipulator_link_data_[4]->mass_    = 1.236;
    manipulator_link_data_[4]->relative_position_ = robotis_framework::getTransitionXYZ(0.19500, -0.05750, 0.03000);
    manipulator_link_data_[4]->joint_axis_        = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    manipulator_link_data_[4]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[4]->joint_limit_max_   =  1    * M_PI;
    manipulator_link_data_[4]->joint_limit_min_   = -0.65 * M_PI;
    manipulator_link_data_[4]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

    manipulator_link_data_[5]->name_    = "joint5";
    manipulator_link_data_[5]->parent_  = 4;
    manipulator_link_data_[5]->sibling_ = -1;
    manipulator_link_data_[5]->child_   = 6;
    manipulator_link_data_[5]->mass_    = 0.491;
    manipulator_link_data_[5]->relative_position_ = robotis_framework::getTransitionXYZ(0.06300, 0.04500, 0.00000);
    manipulator_link_data_[5]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    manipulator_link_data_[5]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[5]->joint_limit_max_   =  0.9 * M_PI;
    manipulator_link_data_[5]->joint_limit_min_   = -0.9 * M_PI;
    manipulator_link_data_[5]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

    manipulator_link_data_[6]->name_    = "joint6";
    manipulator_link_data_[6]->parent_  = 5;
    manipulator_link_data_[6]->sibling_ = -1;
    manipulator_link_data_[6]->child_   = 7;
    manipulator_link_data_[6]->mass_    = 0.454;
    manipulator_link_data_[6]->relative_position_ = robotis_framework::getTransitionXYZ(0.12300, -0.04500, 0.00000);
    manipulator_link_data_[6]->joint_axis_        = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    manipulator_link_data_[6]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[6]->joint_limit_max_   =  0.5 * M_PI;
    manipulator_link_data_[6]->joint_limit_min_   = -0.5 * M_PI;
    manipulator_link_data_[6]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

    manipulator_link_data_[7]->name_    = "joint7";
    manipulator_link_data_[7]->parent_  = 6;
    manipulator_link_data_[7]->sibling_ = -1;
    manipulator_link_data_[7]->child_   = 8;
    manipulator_link_data_[7]->mass_    = 0.454;
    manipulator_link_data_[7]->relative_position_ = robotis_framework::getTransitionXYZ(0.12300, -0.04500, 0.00000);
    manipulator_link_data_[7]->joint_axis_        = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    manipulator_link_data_[7]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[7]->joint_limit_max_   =  0.9 * M_PI;
    manipulator_link_data_[7]->joint_limit_min_   = -0.9 * M_PI;
    manipulator_link_data_[7]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

    manipulator_link_data_[8]->name_    = "end";
    manipulator_link_data_[8]->parent_  = 7;
    manipulator_link_data_[8]->sibling_ = -1;
    manipulator_link_data_[8]->child_   = -1;
    manipulator_link_data_[8]->mass_    = 0.0;
    manipulator_link_data_[8]->relative_position_ = robotis_framework::getTransitionXYZ(0.0115, 0.0, 0.0);
    manipulator_link_data_[8]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[8]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[8]->joint_limit_max_   = 100.0;
    manipulator_link_data_[8]->joint_limit_min_   = -100.0;
    manipulator_link_data_[8]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);
  }

  gen_DHLinksTable();
}

std::vector<int> ManipulatorKinematicsDynamics::findRoute(int to)
{
  int id = manipulator_link_data_[to]->parent_;

  std::vector<int> idx;

  if (id == 0)
  {
    idx.push_back(0);
    idx.push_back(to);
  }
  else
  {
    idx = findRoute(id);
    idx.push_back(to);
  }

  return idx;
}

std::vector<int> ManipulatorKinematicsDynamics::findRoute(int from, int to)
{
  int id = manipulator_link_data_[to]->parent_;

  std::vector<int> idx;

  if (id == from)
  {
    idx.push_back(from);
    idx.push_back(to);
  }
  else if (id != 0)
  {
    idx = findRoute(from, id);
    idx.push_back(to);
  }

  return idx;
}

double ManipulatorKinematicsDynamics::totalMass(int joint_ID)
{
  double mass;

  if (joint_ID == -1)
    mass = 0.0;
  else
    mass = manipulator_link_data_[joint_ID]->mass_ + totalMass(manipulator_link_data_[joint_ID]->sibling_)
                                                   + totalMass(manipulator_link_data_[joint_ID]->child_);

  return mass;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::calcMC(int joint_ID)
{
  Eigen::MatrixXd mc(3, 1);

  if (joint_ID == -1)
    mc = Eigen::MatrixXd::Zero(3, 1);
  else
  {
    mc = manipulator_link_data_[joint_ID]->mass_ * (manipulator_link_data_[joint_ID]->orientation_
        * manipulator_link_data_[joint_ID]->center_of_mass_ + manipulator_link_data_[joint_ID]->position_);
    mc = mc + calcMC(manipulator_link_data_[joint_ID]->sibling_) + calcMC(manipulator_link_data_[joint_ID]->child_);
  }

  return mc;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::calcCOM(Eigen::MatrixXd MC)
{
  double mass;
  Eigen::MatrixXd COM(3, 1);

  mass = totalMass(0);

  COM = MC / mass;

  return COM;
}

void ManipulatorKinematicsDynamics::forwardKinematics(int joint_ID)
{
  if (joint_ID == -1)
    return;

  manipulator_link_data_[0]->position_ = Eigen::MatrixXd::Zero(3, 1);
  manipulator_link_data_[0]->orientation_ = robotis_framework::calcRodrigues(
                                                robotis_framework::calcHatto(manipulator_link_data_[0]->joint_axis_),
                                                manipulator_link_data_[0]->joint_angle_
                                            );

  if (joint_ID != 0)
  {
    int parent = manipulator_link_data_[joint_ID]->parent_;

    manipulator_link_data_[joint_ID]->position_ = manipulator_link_data_[parent]->orientation_
                                                  * manipulator_link_data_[joint_ID]->relative_position_
                                                  + manipulator_link_data_[parent]->position_;
    manipulator_link_data_[joint_ID]->orientation_ = manipulator_link_data_[parent]->orientation_
                                                    * robotis_framework::calcRodrigues(robotis_framework::calcHatto(manipulator_link_data_[joint_ID]->joint_axis_),
                                                                                       manipulator_link_data_[joint_ID]->joint_angle_);

    manipulator_link_data_[joint_ID]->transformation_.block<3, 1>(0, 3) = manipulator_link_data_[joint_ID]->position_;
    manipulator_link_data_[joint_ID]->transformation_.block<3, 3>(0, 0) = manipulator_link_data_[joint_ID]->orientation_;
  }

  forwardKinematics(manipulator_link_data_[joint_ID]->sibling_);
  forwardKinematics(manipulator_link_data_[joint_ID]->child_);
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::calcJacobian(std::vector<int> idx)
{
  int idx_size = idx.size();
  int end = idx_size - 1;

  Eigen::MatrixXd tar_position = manipulator_link_data_[idx[end]]->position_;
  Eigen::MatrixXd Jacobian = Eigen::MatrixXd::Zero(6, idx_size);

  for (int index = 0; index < idx_size; index++)
  {
    int id = idx[index];

    Eigen::MatrixXd tar_orientation = manipulator_link_data_[id]->orientation_ * manipulator_link_data_[id]->joint_axis_;

    Jacobian.block(0, index, 3, 1) = robotis_framework::calcCross(tar_orientation,
                                                                  tar_position - manipulator_link_data_[id]->position_);
    Jacobian.block(3, index, 3, 1) = tar_orientation;
  }

  return Jacobian;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::calcJacobianCOM(std::vector<int> idx)
{
  int idx_size = idx.size();
  int end = idx_size - 1;

  Eigen::MatrixXd target_position = manipulator_link_data_[idx[end]]->position_;
  Eigen::MatrixXd jacobianCOM = Eigen::MatrixXd::Zero(6, idx_size);

  for (int index = 0; index < idx_size; index++)
  {
    int     id    = idx[index];
    double  mass  = totalMass(id);

    Eigen::MatrixXd og = calcMC(id) / mass - manipulator_link_data_[id]->position_;
    Eigen::MatrixXd target_orientation = manipulator_link_data_[id]->orientation_ * manipulator_link_data_[id]->joint_axis_;

    jacobianCOM.block(0, index, 3, 1) = robotis_framework::calcCross(target_orientation, og);
    jacobianCOM.block(3, index, 3, 1) = target_orientation;
  }

  return jacobianCOM;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position,
                                                         Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation)
{
  Eigen::MatrixXd pos_err = tar_position - curr_position;
  Eigen::MatrixXd ori_err1 = curr_orientation.inverse() * tar_orientation;
  Eigen::MatrixXd ori_err2 = curr_orientation * robotis_framework::convertRotToOmega(ori_err1);

  Eigen::MatrixXd err = Eigen::MatrixXd::Zero(6, 1);
  err.block(0, 0, 3, 1) = pos_err;
  err.block(3, 0, 3, 1) = ori_err2;

  return err;
}

bool ManipulatorKinematicsDynamics::inverseKinematics(int to, Eigen::MatrixXd tar_position,
    Eigen::MatrixXd tar_orientation, int max_iter, double ik_err)
{
  bool ik_success     = false;
  bool limit_success  = false;

  forwardKinematics(0);

  std::vector<int> idx = findRoute(to);

  for (int iter = 0; iter < max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);

    Eigen::MatrixXd curr_position     = manipulator_link_data_[to]->position_;
    Eigen::MatrixXd curr_orientation  = manipulator_link_data_[to]->orientation_;

    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm() < ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd jacobian2 = jacobian * jacobian.transpose();
    Eigen::MatrixXd jacobian3 = jacobian.transpose() * jacobian2.inverse();

    Eigen::MatrixXd _delta_angle = jacobian3 * err;

    for (int id = 0; id < idx.size(); id++)
    {
      int joint_num = idx[id];
      manipulator_link_data_[joint_num]->joint_angle_ += _delta_angle.coeff(id);
    }

    forwardKinematics(0);
  }

  for (int id = 0; id < idx.size(); id++)
  {
    int joint_num = idx[id];

    if (manipulator_link_data_[joint_num]->joint_angle_ >= manipulator_link_data_[joint_num]->joint_limit_max_)
    {
      limit_success = false;
      break;
    }
    else if (manipulator_link_data_[joint_num]->joint_angle_ <= manipulator_link_data_[joint_num]->joint_limit_min_)
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    return false;
}

bool ManipulatorKinematicsDynamics::inverseKinematics(int from, int to, Eigen::MatrixXd tar_position,
                                                      Eigen::MatrixXd tar_orientation, int max_iter, double ik_err)
{
  bool ik_success = false;
  bool limit_success = false;

  forwardKinematics(0);

  std::vector<int> idx = findRoute(from, to);

  for (int iter = 0; iter < max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);

    Eigen::MatrixXd curr_position     = manipulator_link_data_[to]->position_;
    Eigen::MatrixXd curr_orientation  = manipulator_link_data_[to]->orientation_;

    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm() < ik_err)
    {
      ik_success = true;
      break;
    }
    else
    {
      ik_success = false;
    }

    Eigen::MatrixXd jacobian2 = jacobian * jacobian.transpose();
    Eigen::MatrixXd jacobian3 = jacobian.transpose() * jacobian2.inverse();

    Eigen::MatrixXd delta_angle = jacobian3 * err;

    for (int id = 0; id < idx.size(); id++)
    {
      int joint_num = idx[id];
      manipulator_link_data_[joint_num]->joint_angle_ += delta_angle.coeff(id);
    }

    forwardKinematics(0);
  }

  for (int id = 0; id < idx.size(); id++)
  {
    int joint_num = idx[id];

    if (manipulator_link_data_[joint_num]->joint_angle_ >= manipulator_link_data_[joint_num]->joint_limit_max_)
    {
      limit_success = false;
      break;
    }
    else if (manipulator_link_data_[joint_num]->joint_angle_ <= manipulator_link_data_[joint_num]->joint_limit_min_)
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    return false;
}

/* ===================================== Evo Kinematics Begin ===================================== */
void ManipulatorKinematicsDynamics::fk()
{
    /* get angle of joints */
    Eigen::VectorXd angle(7);
    for (int i = 0; i < MAX_JOINT_ID; i++)
    {
        angle[i] = manipulator_link_data_[i+1]->joint_angle_;
    }
        
    /* robotis的配置 角度轉換成 evo配置的 */
    angle[1]  = -angle[1];
    angle[1] += M_PI_2;
    //angle[3] += M_PI_2;
    angle[5]  = -angle[5];

    /* 齊次矩陣 */
    Eigen::Matrix4d T0_6 = Eigen::Matrix4d::Identity();
    for (int i = 0; i < angle.size(); i++)
    {
        Eigen::Matrix4d A;
        Gen_TFMat(i, angle[i], A);
        T0_6 *= A;
    }

    /* get position and mat of rotation */
    Eigen::Vector3d pos = T0_6.block<3, 1>(0, 3);
    Eigen::Matrix3d ori = T0_6.block<3, 3>(0, 0);

/* --------------------------------------------- position -------------------------------------------- */
    /* update position */
    manipulator_link_data_[END_LINK]->position_ = pos;
    //std::cout << "fk x,y,z:\n" << pos << std::endl;

/* ------------------------------------------- orientation ------------------------------------------- */
    double pitch = atan2(ori(2, 2), sqrt(1 - pow(ori(2, 2), 2)));  // pitch
    double roll, yaw;

    if (abs(ori(2, 2)) < 0.9999)
    {
        double Cz =  ori(1, 2) / sqrt(1 - pow(ori(2, 2), 2));
        double Sz = -ori(0, 2) / sqrt(1 - pow(ori(2, 2), 2));
        yaw = atan2(Sz, Cz);    // yaw

        double Cy =  ori(2, 0) / sqrt(1 - pow(ori(2, 2), 2));
        double Sy = -ori(2, 1) / sqrt(1 - pow(ori(2, 2), 2));
        roll = atan2(Sy, Cy);   // roll
    }
    else
    {
        yaw = angle[0];         // yaw

        roll = (atan2(ori(1, 1), ori(0, 1)) - yaw);  // roll
        roll = pitch > 0? roll: -roll;

        if (roll > M_PI)
            roll -= M_2_PI;
        else if (roll < -M_PI)
            roll += M_2_PI;
    }

    /* update matrix of rotation */
    manipulator_link_data_[END_LINK]->orientation_ = robotis_framework::convertRPYToRotation(roll, pitch, yaw);
    //std::cout << "fk p,r,y:\n" << pitch * 180.0 / M_PI << " " << roll * 180.0 / M_PI << " " << yaw * 180.0 / M_PI << std::endl;
}

bool ManipulatorKinematicsDynamics::ik(Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation)
{
    Eigen::VectorXd angle(7);
    Eigen::MatrixXd rpy = robotis_framework::convertRotationToRPY(tar_orientation);

    double pitch = rpy(1, 0);
    double roll  = rpy(0, 0);
    double yaw   = rpy(2, 0);

    std::cout << "ik recv x,y,z:\n" << tar_position << std::endl;
    std::cout << "ik recv r,p,y:\n" << rpy * 180 / M_PI << std::endl;
    
    double Cx = cos(pitch);
    double Sx = sin(pitch);
    double Cy = cos(roll);
    double Sy = sin(roll);
    double Cz = cos(yaw);
    double Sz = sin(yaw);
    
    /* desired cmd */
    Eigen::Vector3d position = tar_position;
    Eigen::Matrix3d RPY_Rot;                        // orientation
    RPY_Rot << Cz*Sy + Sz*Sx*Cy,  Cz*Cy - Sz*Sx*Sy, -Sz*Cx,
               Sz*Sy - Cz*Sx*Cy,  Sz*Cy + Cz*Sx*Sy,  Cz*Cx,
               Cx*Cy,            -Cx*Sy,             Sx;

    //std::cout << "rot mat:\n" << RPY_Rot << std::endl;

    int Elbow = -1;                                  // Elbow Up = -1, Elbow Down =  1
    int Wrist = -1;                                  // Wrist Up =  1, Wrist Down = -1
    
    double L1 = DH(0, 2);
    double L3 = DH(2, 0);
    double L4 = DH(4, 2);
    double L5 = DH(6, 2);
    double L2 = sqrt(pow(DH(2, 2), 2) + pow(L3, 2));
    double F  = DH(2, 2) + DH(4, 2);

    /* --------------------------------------------- position -------------------------------------------- */
    Eigen::Vector3d Hand_pose = L5 * RPY_Rot.block<3, 1>(0, 2);
    Eigen::Vector3d WristPos = tar_position - Hand_pose;

    /* joint 1 */
    angle[0] = (fabs(WristPos(0)) < 0.000001 && fabs(WristPos(1)) < 0.000001) ?
        0.12345: atan2(-WristPos(0) , WristPos(1));

    double xc_2 = pow(WristPos(0), 2);
    double yc_2 = pow(WristPos(1), 2);

    double D = (WristPos - Eigen::Vector3d(0, 0, L1)).norm();
    double E = (WristPos - Eigen::Vector3d(0, 0, 0)).norm();

    double L34 = sqrt(pow(L3, 2) + pow(L4, 2));

    double Alpha = acos((pow(L2, 2) + pow(D, 2) - pow(L34, 2)) / (2 * L2 * D));
    double Gamma = acos((pow(L1, 2) + pow(D, 2) - pow(E, 2))   / (2 * L1 * D));

    /* 偏軸的偏移角度 */
    double Beta  = 83.52 * M_PI / 180.0;
    //double Lunda = 6.48  * M_PI / 180.0;
    
    double Epslon = acos( (pow(L2, 2) + pow(L34, 2) - pow(D, 2)) / (2 * L2 * L34));
    double Fai    = acos( (pow(L2, 2) + pow(L34, 2) - pow(F, 2)) / (2 * L2 * L34));

    /* joint 2 to 4 */
    angle[1] = Alpha + (Gamma - Beta);    
    angle[3] = (360.0 * M_PI / 180.0) - (Epslon + Fai); 
    angle[2] = 0;

    /* ------------------------------------------- orientation ------------------------------------------- */
    double sida1 = angle[0] + DH(0, 3);
    double sida2 = angle[1] + DH(1, 3);
    double sida3 = angle[2] + DH(2, 3);
    double sida4 = angle[3] + DH(3, 3);
    
    double c1 = cos(sida1);  
    double s1 = sin(sida1);
    double c2 = cos(sida2);
    double s2 = sin(sida2);
    double c4 = cos(sida4);
    double s4 = sin(sida4);

    Eigen::Matrix3d R0_4;
    R0_4 << -c1*s2*s4 - c1*c2*c4,  s1, c1*c2*s4 - c1*c4*s2,
            -s1*s2*s4 - c2*c4*s1, -c1, c2*s1*s4 - c4*s1*s2,
             c2*s4 - c4*s2,         0, c2*c4 + s2*s4;

    Eigen::Matrix3d R4_6 = R0_4.transpose() * RPY_Rot;

    /* joint 6 */
    angle[5] = atan2(Wrist * sqrt(1 - pow(R4_6(2, 2), 2)), R4_6(2, 2));
    
    if (abs(R4_6(2, 2)) > 0.9999)
    {
        /* joint 5, 7 */
        angle[4] = 0;
        angle[6] = atan2(R4_6(1, 0) , R4_6(0, 0));
    }
    else
    {
        if (Wrist > 0)
        {
            angle[4] = atan2(R4_6(1,2),  R4_6(0,2));
            angle[6] = atan2(R4_6(2,1), -R4_6(2,0));
            if (abs(angle[4]) > 120.0 * M_PI / 180.0)
            {
                angle[5] = atan2(-Wrist * sqrt(1 - pow(R4_6(2, 2), 2)), R4_6(2, 2));
                angle[4] = atan2(-R4_6(1,2) , -R4_6(0,2));
                angle[6] = atan2(-R4_6(2,1) ,  R4_6(2,0));
            }
        }
        else
        {
            angle[4] = atan2(-R4_6(1,2), -R4_6(0,2));
            angle[6] = atan2(-R4_6(2,1),  R4_6(2,0));
            if(abs(angle[4]) > 120.0 * M_PI / 180.0)
            {
                angle[5] = atan2(-Wrist * sqrt(1 - pow(R4_6(2, 2), 2)), R4_6(2, 2));
                angle[4] = atan2(R4_6(1,2) ,  R4_6(0,2));
                angle[6] = atan2(R4_6(2,1) , -R4_6(2,0));
            }
        }
    }

    // for (int i = 0; i < 7; i++)
    //    std::cout << "ik angle " << i << ": "<< angle[i] * 180.0 / M_PI << std::endl;

    /* evo配置算出來的角度轉換成robotis的配置 */
    angle[1] -= M_PI_2;
    angle[1]  = -angle[1];
    //angle[3] -= M_PI_2;
    angle[5]  = -angle[5]; 

    /* 存起解出的關節角度 */
    for (int i = 0; i < MAX_JOINT_ID; i++)
    {
        // std::cout << "cvt ik angle " << i << ": "<< angle[i] * 180.0 / M_PI << std::endl;

        /* checking angle is nan */
        if (std::isnan(angle[i]))
            return false;

        manipulator_link_data_[i+1]->joint_angle_ = angle[i];
    }

    /* checking joint limit */
    for (int joint_num = 1; joint_num <= MAX_JOINT_ID; joint_num++)
    {
        LinkData& link_data = *manipulator_link_data_[joint_num];

        if (link_data.joint_angle_ > link_data.joint_limit_max_ ||
            link_data.joint_angle_ < link_data.joint_limit_min_)
        {
            std::cout << "ik joint limit: " << joint_num << std::endl;
            return false;
        }
    }
    return true;
}

void ManipulatorKinematicsDynamics::gen_DHLinksTable()
{
    DH.resize(7, 4);

    /* a alpha d delta */
    DH <<    0,  M_PI_2,  0.159, M_PI_2,
             0,  M_PI_2,      0, M_PI_2,
          0.03, -M_PI_2,  0.264,   M_PI,
          0.03, -M_PI_2,      0,   M_PI,
             0, -M_PI_2,  0.258,      0,
             0,  M_PI_2,      0,      0,
             0,       0, 0.1345,      0;
}

void ManipulatorKinematicsDynamics::Gen_TFMat(int index, double theta, Eigen::Matrix4d& A)
{
    double c_theta = cos(theta + DH(index, 3));
    double s_theta = sin(theta + DH(index, 3));
    double c_alpha = cos(DH(index, 1));
    double s_alpha = sin(DH(index, 1));
    
    A << c_theta, -s_theta*c_alpha,  s_theta*s_alpha, DH(index, 0) * c_theta,
         s_theta,  c_theta*c_alpha, -c_theta*s_alpha, DH(index, 0) * s_theta,
         0,        s_alpha,          c_alpha,         DH(index, 2),
         0,        0,                0,               1;
}
/*  ===================================== Evo Kinematics End ===================================== */

}
