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

#define ROUND_DIGITS 8

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
    fai = 0;
    
    for (int id = 0; id <= ALL_JOINT_ID; id++)
        manipulator_link_data_[id] = new LinkData();

    if (tree == ARM)
    {
        manipulator_link_data_[0]->name_ = "base";
        manipulator_link_data_[0]->parent_ = -1;
        manipulator_link_data_[0]->sibling_ = -1;
        manipulator_link_data_[0]->child_ = 1;
        manipulator_link_data_[0]->mass_ = 0.0;
        manipulator_link_data_[0]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
        manipulator_link_data_[0]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
        manipulator_link_data_[0]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
        manipulator_link_data_[0]->joint_limit_max_ = 100.0;
        manipulator_link_data_[0]->joint_limit_min_ = -100.0;
        manipulator_link_data_[0]->inertia_ = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        manipulator_link_data_[1]->name_ = "joint1";
        manipulator_link_data_[1]->parent_ = 0;
        manipulator_link_data_[1]->sibling_ = -1;
        manipulator_link_data_[1]->child_ = 2;
        manipulator_link_data_[1]->mass_ = 0.85644;
        manipulator_link_data_[1]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.126);
        manipulator_link_data_[1]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 1.0);
        manipulator_link_data_[1]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
        manipulator_link_data_[1]->joint_limit_max_ =  0.95 * M_PI;
        manipulator_link_data_[1]->joint_limit_min_ = -0.95 * M_PI;
        manipulator_link_data_[1]->inertia_ = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

        manipulator_link_data_[2]->name_ = "joint2";
        manipulator_link_data_[2]->parent_ = 1;
        manipulator_link_data_[2]->sibling_ = -1;
        manipulator_link_data_[2]->child_ = 3;
        manipulator_link_data_[2]->mass_ = 0.94658;
        manipulator_link_data_[2]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.06900, 0.033);
        manipulator_link_data_[2]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
        manipulator_link_data_[2]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
        manipulator_link_data_[2]->joint_limit_max_ = 0.5 * M_PI;
        manipulator_link_data_[2]->joint_limit_min_ = -0.5 * M_PI;
        manipulator_link_data_[2]->inertia_ = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

        manipulator_link_data_[3]->name_ = "joint3";
        manipulator_link_data_[3]->parent_ = 2;
        manipulator_link_data_[3]->sibling_ = -1;
        manipulator_link_data_[3]->child_ = 4;
        manipulator_link_data_[3]->mass_ = 1.30260;
        manipulator_link_data_[3]->relative_position_ = robotis_framework::getTransitionXYZ(0.03000, -0.01150, 0.26400);
        manipulator_link_data_[3]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
        manipulator_link_data_[3]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
        manipulator_link_data_[3]->joint_limit_max_ =  0.95 * M_PI;
        manipulator_link_data_[3]->joint_limit_min_ = -0.95 * M_PI;
        manipulator_link_data_[3]->inertia_ = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

        manipulator_link_data_[4]->name_ = "joint4";
        manipulator_link_data_[4]->parent_ = 3;
        manipulator_link_data_[4]->sibling_ = -1;
        manipulator_link_data_[4]->child_ = 5;
        manipulator_link_data_[4]->mass_ = 1.236;
        manipulator_link_data_[4]->relative_position_ = robotis_framework::getTransitionXYZ(0.19500, -0.05750, 0.03000);
        manipulator_link_data_[4]->joint_axis_ = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
        manipulator_link_data_[4]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
        manipulator_link_data_[4]->joint_limit_max_ =     1 * M_PI;
        manipulator_link_data_[4]->joint_limit_min_ = -0.65 * M_PI;
        manipulator_link_data_[4]->inertia_ = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

        manipulator_link_data_[5]->name_ = "joint5";
        manipulator_link_data_[5]->parent_ = 4;
        manipulator_link_data_[5]->sibling_ = -1;
        manipulator_link_data_[5]->child_ = 6;
        manipulator_link_data_[5]->mass_ = 0.491;
        manipulator_link_data_[5]->relative_position_ = robotis_framework::getTransitionXYZ(0.06300, 0.04500, 0.00000);
        manipulator_link_data_[5]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
        manipulator_link_data_[5]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
        manipulator_link_data_[5]->joint_limit_max_ =  0.95 * M_PI;
        manipulator_link_data_[5]->joint_limit_min_ = -0.95 * M_PI;
        manipulator_link_data_[5]->inertia_ = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

        manipulator_link_data_[6]->name_ = "joint6";
        manipulator_link_data_[6]->parent_ = 5;
        manipulator_link_data_[6]->sibling_ = -1;
        manipulator_link_data_[6]->child_ = 7;
        manipulator_link_data_[6]->mass_ = 0.454;
        manipulator_link_data_[6]->relative_position_ = robotis_framework::getTransitionXYZ(0.12300, -0.04500, 0.00000);
        manipulator_link_data_[6]->joint_axis_ = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
        manipulator_link_data_[6]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
        manipulator_link_data_[6]->joint_limit_max_ =  0.5 * M_PI;
        manipulator_link_data_[6]->joint_limit_min_ = -0.5 * M_PI;
        manipulator_link_data_[6]->inertia_ = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

        manipulator_link_data_[7]->name_ = "joint7";
        manipulator_link_data_[7]->parent_ = 6;
        manipulator_link_data_[7]->sibling_ = -1;
        manipulator_link_data_[7]->child_ = 8;
        manipulator_link_data_[7]->mass_ = 0.454;
        manipulator_link_data_[7]->relative_position_ = robotis_framework::getTransitionXYZ(0.12300, -0.04500, 0.00000);
        manipulator_link_data_[7]->joint_axis_ = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
        manipulator_link_data_[7]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
        manipulator_link_data_[7]->joint_limit_max_ =  0.95 * M_PI;
        manipulator_link_data_[7]->joint_limit_min_ = -0.95 * M_PI;
        manipulator_link_data_[7]->inertia_ = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

        manipulator_link_data_[8]->name_ = "end";
        manipulator_link_data_[8]->parent_ = 7;
        manipulator_link_data_[8]->sibling_ = -1;
        manipulator_link_data_[8]->child_ = -1;
        manipulator_link_data_[8]->mass_ = 0.0;
        manipulator_link_data_[8]->relative_position_ = robotis_framework::getTransitionXYZ(0.0115, 0.0, 0.0);
        manipulator_link_data_[8]->joint_axis_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
        manipulator_link_data_[8]->center_of_mass_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
        manipulator_link_data_[8]->joint_limit_max_ = 100.0;
        manipulator_link_data_[8]->joint_limit_min_ = -100.0;
        manipulator_link_data_[8]->inertia_ = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);
    }

    load_LinkParam();
    std::cout << "DH:\n" << DH << std::endl;
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
        mass = manipulator_link_data_[joint_ID]->mass_ + totalMass(manipulator_link_data_[joint_ID]->sibling_) + totalMass(manipulator_link_data_[joint_ID]->child_);

    return mass;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::calcMC(int joint_ID)
{
    Eigen::MatrixXd mc(3, 1);

    if (joint_ID == -1)
        mc = Eigen::MatrixXd::Zero(3, 1);
    else
    {
        mc = manipulator_link_data_[joint_ID]->mass_ * (manipulator_link_data_[joint_ID]->orientation_ * manipulator_link_data_[joint_ID]->center_of_mass_ + manipulator_link_data_[joint_ID]->position_);
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
        manipulator_link_data_[0]->joint_angle_);

    if (joint_ID != 0)
    {
        int parent = manipulator_link_data_[joint_ID]->parent_;

        manipulator_link_data_[joint_ID]->position_ = manipulator_link_data_[parent]->orientation_ * manipulator_link_data_[joint_ID]->relative_position_ + manipulator_link_data_[parent]->position_;
        manipulator_link_data_[joint_ID]->orientation_ = manipulator_link_data_[parent]->orientation_ * robotis_framework::calcRodrigues(robotis_framework::calcHatto(manipulator_link_data_[joint_ID]->joint_axis_),
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
        int id = idx[index];
        double mass = totalMass(id);

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
    bool ik_success = false;
    bool limit_success = false;

    forwardKinematics(0);

    std::vector<int> idx = findRoute(to);

    for (int iter = 0; iter < max_iter; iter++)
    {
        Eigen::MatrixXd jacobian = calcJacobian(idx);

        Eigen::MatrixXd curr_position = manipulator_link_data_[to]->position_;
        Eigen::MatrixXd curr_orientation = manipulator_link_data_[to]->orientation_;

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

        Eigen::MatrixXd curr_position = manipulator_link_data_[to]->position_;
        Eigen::MatrixXd curr_orientation = manipulator_link_data_[to]->orientation_;

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
    Eigen::VectorXd angle(MAX_JOINT_ID);
    for (int i = 0; i < MAX_JOINT_ID; i++)
    {
        angle[i] = manipulator_link_data_[i + 1]->joint_angle_;
    }

    /* robotis的配置 角度轉換成 evo配置的 */
    angle[1] = -angle[1];
    angle[1] += M_PI_2;
    //angle[3] += M_PI_2;
    angle[5] = -angle[5];

    /* 齊次矩陣 */
    Eigen::Matrix4d T0_6 = Eigen::Matrix4d::Identity();

    std::vector<Eigen::Vector3d> jointPos;
    jointPos.push_back(T0_6.block<3, 1>(0, 3));

    for (int i = 0; i < angle.size(); i++)
    {
        Eigen::Matrix4d A;
        gen_TFMat(i, angle[i], A);
        T0_6 *= A;

        jointPos.push_back(T0_6.block<3, 1>(0, 3));
    }

    /* get position and mat of rotation */
    Eigen::Vector3d pos = T0_6.block<3, 1>(0, 3);
    Eigen::Matrix3d ori = T0_6.block<3, 3>(0, 0);

    /* --------------------------------------------- position -------------------------------------------- */
    /* update position */
    manipulator_link_data_[END_LINK]->position_ = pos;

    /* ------------------------------------------- orientation ------------------------------------------- */
    /* fixed value range */
    if (fabs(ori(2, 2)) > 1)
        ori(2, 2) = ori(2, 2) > 0 ? 1 : -1;

    double pitch = atan2(ori(2, 2), sqrt(1 - pow(ori(2, 2), 2))); // pitch
    double roll, yaw;

    /* pitch != +-90 deg */
    if (fabs(ori(2, 2)) <= 1.0 - pow(10, -8))
    {
        double Cz =  ori(1, 2) / sqrt(1 - pow(ori(2, 2), 2));
        double Sz = -ori(0, 2) / sqrt(1 - pow(ori(2, 2), 2));
        yaw = atan2(Sz, Cz); // yaw

        double Cy =  ori(2, 0) / sqrt(1 - pow(ori(2, 2), 2));
        double Sy = -ori(2, 1) / sqrt(1 - pow(ori(2, 2), 2));
        roll = atan2(Sy, Cy); // roll
    }
    else
    {
        double tmp_Roll = atan2(-ori(2, 0), sqrt(pow(ori(0, 0), 2) + pow(ori(1, 0), 2)));
        double a = ori(1, 0) / cos(tmp_Roll);
        double b = ori(0, 0) / cos(tmp_Roll);

        yaw  = atan2(a, b);
        yaw += ori(2, 2) == 1? M_PI_2: -M_PI_2;
        normalizeAngle(yaw);

        roll = 0;
    }

    /* update matrix of rotation */
    manipulator_link_data_[END_LINK]->orientation_ = robotis_framework::convertRPYToRotation(roll, pitch, yaw);

    // std::cout << "fk x,y,z:\n" << pos << std::endl;
    // std::cout << "fk p,r,y:\n" << pitch * 180.0 / M_PI << " " << roll * 180.0 / M_PI << " " << yaw * 180.0 / M_PI << std::endl;

    /* claculate redundancy for fai */
    this->fai = cal_Redundancy(jointPos);
}

bool ManipulatorKinematicsDynamics::ik(Eigen::MatrixXd& tar_position, Eigen::MatrixXd& tar_orientation, double tarFai /* = 0 */)
{
    Eigen::VectorXd angle(7);
    Eigen::MatrixXd rpy = robotis_framework::convertRotationToRPY(tar_orientation);

    double pitch = rpy(1, 0);
    double roll  = rpy(0, 0);
    double yaw   = rpy(2, 0);

    // std::cout << "ik recv x,y,z:\n" << tar_position << std::endl;
    // std::cout << "ik recv r,p,y:\n" << rpy * 180 / M_PI << std::endl;
    // std::cout << "ik recv fai: " << tarFai * 180 / M_PI << std::endl;

    double Cx = cos(pitch);
    double Sx = sin(pitch);
    double Cy = cos(roll);
    double Sy = sin(roll);
    double Cz = cos(yaw);
    double Sz = sin(yaw);

    /* desired cmd */
    Eigen::Vector3d position = tar_position;
    Eigen::Matrix3d RPY_Rot; // orientation
    RPY_Rot << Cz*Sy + Sz*Sx*Cy, Cz*Cy - Sz*Sx*Sy, -Sz*Cx,
               Sz*Sy - Cz*Sx*Cy, Sz*Cy + Cz*Sx*Sy,  Cz*Cx,
                          Cx*Cy,           -Cx*Sy,     Sx;

    double d_bs = DH(0, 2);
    double d_se = DH(2, 2);
    double d_ew = DH(4, 2);
    double d_wt = DH(6, 2);

    /* --------------------------------------------- position -------------------------------------------- */
    Eigen::Vector3d P_s(0, 0, d_bs);
    Eigen::Vector4d P_w2(0, 0, -d_wt, 1);

    Eigen::MatrixXd T0_7(3, 4);
    T0_7.block<3, 1>(0, 3) = position;
    T0_7.block<3, 3>(0, 0) = RPY_Rot;

    Eigen::Vector3d P_w = T0_7 * P_w2;

    Eigen::Vector3d P_e;
    Eigen::Vector3d P_LJ;
    cal_ElbowInfo(P_s, P_w, tarFai, P_e, P_LJ);

    /* Calculate Joint Angle 4 */
    Eigen::Vector3d AB = P_e - P_s;
    Eigen::Vector3d BC = P_w - P_e;
    double r = cal_VecIncAngle(AB, BC);
    angle[3] = r + rho2;

    /* Calculate Joint Angle 1, 2 */
    double a = P_LJ(2) - d_bs;
    double b = L_sl;

    if(P_LJ(1) >= 0)//0.0001)
    {
        if (P_w(1) < 0)//0.0001)      //(LJ, W) = (+, -)
        {
            angle[0] = atan2(-P_LJ(0), P_LJ(1)) - M_PI;
            angle[1] = M_PI - asin(a / b);
        }
        else                        //(LJ, W) = (+, +)
        {
            angle[0] = atan2(-P_LJ(0), P_LJ(1));
            angle[1] = asin(a / b);
        }
    }
    else
    {
        if (P_w(1) >= 0)//0.0001)        //(LJ, W) = (-, +)
        {
            angle[0] = atan2(-P_LJ(0), P_LJ(1)) - M_PI;
            angle[1] = M_PI - asin(a / b);
        }
        else                        //(LJ, W) = (-, -)
        {
            angle[0] = atan2(-P_LJ(0), P_LJ(1));
            angle[1] = asin(a / b);
        }
    }

    /* Calculate Joint Angle 3 */
    Eigen::Matrix4d A[2];
    for (int i = 0; i < 2; i++)
    {
        gen_TFMat(i, angle[i], A[i]);
    }

    Eigen::Vector4d P_e4(P_e);
    P_e4(3) = 1;
    Eigen::Vector4d C = A[1].inverse() * A[0].inverse() * P_e4;

    angle[2] = acos(C(0) / sqrt(pow(C(0), 2) + pow(C(1), 2))) - M_PI;
    angle[2] = C(1) > 0.0001 ? angle[2] : -angle[2];

    // /* ------------------------------------------- orientation ------------------------------------------- */
    double sida1 = angle[0] + DH(0, 3);
    double sida2 = angle[1] + DH(1, 3);
    double sida3 = angle[2] + DH(2, 3);
    double sida4 = angle[3] + DH(3, 3);

    Eigen::Matrix3d R1, R2, R3, R4;
    R1 << cos(sida1),  0,  sin(sida1),
          sin(sida1),  0, -cos(sida1),
                   0,  1,           0;

    R2 << cos(sida2),  0,  sin(sida2),
          sin(sida2),  0, -cos(sida2),
                   0,  1,           0;

    R3 << cos(sida3),  0, -sin(sida3),
          sin(sida3),  0,  cos(sida3),
                   0, -1,           0;

    R4 << cos(sida4),  0, -sin(sida4),
          sin(sida4),  0,  cos(sida4),
                   0, -1,           0;

    Eigen::Matrix3d R0_4 = R1 * R2 * R3 * R4;
    Eigen::Matrix3d R4_7 = R0_4.transpose() * RPY_Rot;

    int Wrist = -1; // Wrist Up =  1, Wrist Down = -1

    /* joint 6 */
    angle[5] = atan2(Wrist * sqrt(1 - pow(R4_7(2, 2), 2)), R4_7(2, 2));

    if (fabs(R4_7(2, 2)) > 0.99999)
    {
        /* joint 5, 7 */
        angle[4] = 0;
        angle[6] = atan2(R4_7(1, 0), R4_7(0, 0));
    }
    else
    {
        if (Wrist > 0)
        {
            angle[4] = atan2(R4_7(1, 2),  R4_7(0, 2));
            angle[6] = atan2(R4_7(2, 1), -R4_7(2, 0));
            if (fabs(angle[4]) > 120.0 * M_PI / 180.0)
            {
                angle[5] = atan2(-Wrist * sqrt(1 - pow(R4_7(2, 2), 2)), R4_7(2, 2));
                angle[4] = atan2(-R4_7(1, 2), -R4_7(0, 2));
                angle[6] = atan2(-R4_7(2, 1),  R4_7(2, 0));
            }
        }
        else
        {
            angle[4] = atan2(-R4_7(1, 2), -R4_7(0, 2));
            angle[6] = atan2(-R4_7(2, 1),  R4_7(2, 0));
            if (fabs(angle[4]) > 120.0 * M_PI / 180.0)
            {
                angle[5] = atan2(-Wrist * sqrt(1 - pow(R4_7(2, 2), 2)), R4_7(2, 2));
                angle[4] = atan2(R4_7(1, 2),  R4_7(0, 2));
                angle[6] = atan2(R4_7(2, 1), -R4_7(2, 0));
            }
        }
    }

    // for (int i = 0; i < 7; i++)
    //    std::cout << "ik angle " << i+1 << ": "<< angle[i] * 180.0 / M_PI << std::endl;

    for (int i = 0; i < MAX_JOINT_ID; i++)
    {
        /* checking angle is nan */
        if (std::isnan(angle[i]))
        {
            std::cout << "angle[" << i << "] is nan" << std::endl;
            return false;
        }

        if((fabs(angle[i]) > 0) && (fabs(angle[i]) < 0.0001))
            angle[i] = 0;

        normalizeAngle(angle[i]);
    }

    /* evo配置算出來的角度轉換成robotis的配置 */
    angle[1] -= M_PI_2;
    angle[1] = -angle[1];
    //angle[3] -= M_PI_2;
    angle[5] = -angle[5];

    // for (int i = 0; i < 7; i++)
    //    std::cout << "cvt ik angle " << i+1 << ": "<< angle[i] * 180.0 / M_PI << std::endl;

    for (int i = 0; i < MAX_JOINT_ID; i++)
    {
        LinkData &link_data = *manipulator_link_data_[i+1];

        /* 存起解出的關節角度 */
        link_data.joint_angle_  = angle[i];
        /* saving value of calculation about ik */
        ik_calc_joint_angle_[i] = angle[i];

        /* checking joint limit */
        if (link_data.joint_angle_ > link_data.joint_limit_max_ ||
            link_data.joint_angle_ < link_data.joint_limit_min_)
        {
            std::cout << "ik joint limit: " << i+1 << " " << link_data.joint_angle_ * 180.0 / M_PI << std::endl;
            return false;
        }
    }
    return true;
}

void ManipulatorKinematicsDynamics::cal_ElbowInfo(Eigen::Vector3d& P_s, Eigen::Vector3d& P_w, double Fai, Eigen::Vector3d& P_e,Eigen::Vector3d& P_LJ)
{
    using namespace Eigen;

    double L_sw = (P_w - P_s).norm();
    double    a = pow(L_se, 2) + pow(L_sw, 2) - pow(L_ew, 2);
    double    b = 2 * L_se * L_sw;
    double  a_b = a / b;
    if (fabs(a_b) > 1)
        a_b = a_b > 0 ? 1 : -1;

    double  BAE = acos(a_b);
    
    Vector3d  E = P_s + (P_w - P_s) * L_se * cos(BAE) / L_sw;
    double  low_BAE = BAE + rho1;

    Vector3d  low_E = P_s + (P_w - P_s) * L_sl * cos(low_BAE) / L_sw;

    Matrix4d T0_E     = cal_ElbowRotMatrix(P_s, P_w,     E);
    Matrix4d low_T0_E = cal_ElbowRotMatrix(P_s, P_w, low_E);

    P_e  = cal_ElbowPos(T0_E, BAE, L_se, Fai);
    P_LJ = cal_ElbowPos(low_T0_E, low_BAE, L_sl, Fai);
}

double ManipulatorKinematicsDynamics::cal_Redundancy(std::vector<Eigen::Vector3d>& jointPos)
{
    using namespace Eigen;

    Vector3d P_s = jointPos[2];
    Vector3d P_e = jointPos[3];
    Vector3d P_w = jointPos[6];

    double L_sw = (P_w - P_s).norm();
    
    double BAE = acos((pow(L_se, 2) + pow(L_sw, 2) - pow(L_ew, 2)) / (2 * L_se * L_sw)); 
    Vector3d E = P_s + (P_w - P_s) * L_se * cos(BAE) / L_sw;

    Matrix4d T0_E = cal_ElbowRotMatrix(P_s, P_w, E);
    Vector3d P_e0 = cal_ElbowPos(T0_E, BAE, L_se, 0);

    double Redundancy = cal_VecIncAngle(E, P_e0, P_e);    // cal vector angle

    Vector3d ElbowPos = cal_ElbowPos(T0_E, BAE, L_se, Redundancy);
    if ((ElbowPos - P_e).norm() > 0.01)
    {
        Redundancy *= -1;
    }
    return Redundancy;
}

Eigen::Matrix4d ManipulatorKinematicsDynamics::cal_ElbowRotMatrix(Eigen::Vector3d& P_s, Eigen::Vector3d& P_w, Eigen::Vector3d& E)
{
    Eigen::Vector3d V_sw = P_w - P_s;

    double L_sw = V_sw.norm();
    double m    = sqrt(pow(V_sw(0), 2) + pow(V_sw(1), 2));
    double m_mul_Lsw = m * L_sw;

    double T11 = ( -(V_sw(0) * V_sw(2))) / m_mul_Lsw;
    double T21 = ( -(V_sw(1) * V_sw(2))) / m_mul_Lsw;
    double T31 = (pow(V_sw(1), 2) + pow(V_sw(0), 2)) / m_mul_Lsw;

    double T12 =  V_sw(1) / m;
    double T22 = -V_sw(0) / m;
    double T32 =  0.0;

    double T13 =  V_sw(0) / L_sw;
    double T23 =  V_sw(1) / L_sw;
    double T33 =  V_sw(2) / L_sw;

    Eigen::Matrix4d T0_E;
    T0_E << T11, T12, T13, E(0),
            T21, T22, T23, E(1),
            T31, T32, T33, E(2),
              0,   0,   0,    1;

    return T0_E;
}

Eigen::Vector3d ManipulatorKinematicsDynamics::cal_ElbowPos(Eigen::Matrix4d& RotMatrix, double Angle, double LinkLen, double Fai)
{
    double t = LinkLen * sin(Angle);
    double EB_1_1 = t * cos(Fai);
    double EB_2_1 = t * sin(Fai);

    Eigen::Vector4d EB(EB_1_1, EB_2_1, 0, 1);
    Eigen::Vector4d ElbowPos = RotMatrix * EB;
    return ElbowPos.block<3, 1>(0, 0);
}

double ManipulatorKinematicsDynamics::cal_VecIncAngle(Eigen::Vector3d& v1, Eigen::Vector3d& v2)
{
    double a = roundN(v1.dot(v2), ROUND_DIGITS);
    double b = roundN(v1.norm() * v2.norm(), ROUND_DIGITS);

    /* angle */
    return acos(a / b);
}

double ManipulatorKinematicsDynamics::cal_VecIncAngle(Eigen::Vector3d& origin, Eigen::Vector3d& p1, Eigen::Vector3d& p2)
{
    Eigen::Vector3d v1 = p1 - origin;
    Eigen::Vector3d v2 = p2 - origin;
    double a = roundN(v1.dot(v2), ROUND_DIGITS);
    double b = roundN(v1.norm() * v2.norm(), ROUND_DIGITS);

    /* angle */
    return acos(a / b);
}

void ManipulatorKinematicsDynamics::load_LinkParam()
{
    std::string path = ros::package::getPath("manipulator_h_kinematics_dynamics") + "/config/link_param.yaml";
    
    YAML::Node doc;
    try
    {
        // load yaml
        doc = YAML::LoadFile(path.c_str());
    }
    catch (const std::exception &e)
    {
        std::cout << "Fail to load yaml file." << std::endl;
        return;
    }

    DH.resize(7, 4);

    // parse dh links
    YAML::Node _dh_links = doc["dh_links"];
    for (YAML::iterator _it = _dh_links.begin(); _it != _dh_links.end(); ++_it)
    {
        int link_num = _it->first.as<int>();

        std::map<std::string, double> param = _it->second.as<std::map<std::string, double> >();
        DH(link_num - 1, 0) = param["a"];
        DH(link_num - 1, 1) = param["alpha"] * M_PI / 180.0;
        DH(link_num - 1, 2) = param["d"];
        DH(link_num - 1, 3) = param["theta"] * M_PI / 180.0;
    }

    // parse L_sl
    this->L_sl = doc["L_sl"].as<double>();

    set_LinkParam();
}

void ManipulatorKinematicsDynamics::set_LinkParam()
{
    this->L_se = sqrt(pow(DH(2, 0), 2) + pow(DH(2, 2), 2));;    // length from shoulder to elbow
    this->L_ew = sqrt(pow(DH(3, 0), 2) + pow(DH(4, 2), 2));     // length from elbow to wrist
    this->rho1 = acos(DH(2, 2) / L_se);
    this->rho2 = rho1 + acos(DH(4, 2) / L_ew);
}

void ManipulatorKinematicsDynamics::gen_TFMat(int index, double theta, Eigen::Matrix4d &A)
{
    double c_theta = cos(theta + DH(index, 3));
    double s_theta = sin(theta + DH(index, 3));
    double c_alpha = cos(DH(index, 1));
    double s_alpha = sin(DH(index, 1));

    A << c_theta, -s_theta * c_alpha,  s_theta * s_alpha, DH(index, 0) * c_theta,
         s_theta,  c_theta * c_alpha, -c_theta * s_alpha, DH(index, 0) * s_theta,
               0,            s_alpha,            c_alpha,           DH(index, 2),
               0,                  0,                  0,                      1;
}

/* Normalize Angle between 0 and 360 */
void ManipulatorKinematicsDynamics::normalizeAngle(double& rad)
{
    if (rad > M_PI)
        rad -= M_PI * 2.0;
    else if (rad < -M_PI)
        rad += M_PI * 2.0;
}

double ManipulatorKinematicsDynamics::roundN(double num, int digit /* = 0 */)
{
    double real = round(num);
    double dot  = modf(num, &real);
    dot = dot * pow(10.0, (double)digit);
    dot = round(dot) / pow(10.0, (double)digit);

    return real + dot;
}

/*  ===================================== Evo Kinematics End ===================================== */
}
