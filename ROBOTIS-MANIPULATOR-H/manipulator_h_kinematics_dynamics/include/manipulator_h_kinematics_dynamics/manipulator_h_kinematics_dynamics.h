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

#ifndef MANIPULATOR_KINEMATICS_DYNAMICS_MANIPULATOR_KINEMATICS_DYNAMICS_H_
#define MANIPULATOR_KINEMATICS_DYNAMICS_MANIPULATOR_KINEMATICS_DYNAMICS_H_

#include <vector>
/* for checking value is nan */
#include <cmath>
#include "link_data.h"
#include "manipulator_h_kinematics_dynamics_define.h"

/* for load yaml */
#include <map>
#include <string>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

namespace robotis_manipulator_h
{

enum TreeSelect
{
    ARM
};

class ManipulatorKinematicsDynamics
{
private:
    /* DH table */
    Eigen::MatrixXd DH;
    /* Link Param */
    double L_se;
    double L_sl;
    double L_ew;
    double rho1;
    double rho2;

    double fai;

public:
    ManipulatorKinematicsDynamics();
    ManipulatorKinematicsDynamics(TreeSelect tree);
    ~ManipulatorKinematicsDynamics();

    LinkData *manipulator_link_data_[ALL_JOINT_ID + 1];
    /* saving value of calculation about ik */
    double    ik_calc_joint_angle_[MAX_JOINT_ID];

    std::vector<int> findRoute(int to);
    std::vector<int> findRoute(int from, int to);

    double totalMass(int joint_ID);
    Eigen::MatrixXd calcMC(int joint_ID);
    Eigen::MatrixXd calcCOM(Eigen::MatrixXd MC);

    void forwardKinematics(int joint_ID);

    Eigen::MatrixXd calcJacobian(std::vector<int> idx);
    Eigen::MatrixXd calcJacobianCOM(std::vector<int> idx);
    Eigen::MatrixXd calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position,
                              Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation);

    bool inverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
                           int max_iter, double ik_err);
    bool inverseKinematics(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
                           int max_iter, double ik_err);

    /* ==================== Evo Kinematics ==================== */
    void load_LinkParam();
    void set_LinkParam();
    // set endlink length
    bool set_endlink(double);

    void gen_TFMat(int index, double theta, Eigen::Matrix4d& A);

    inline double get_Redundancy() { return fai; }

    /* ------------------ forward kinematics ------------------ */
    void fk();
    double cal_Redundancy(std::vector<Eigen::Vector3d>& jointPos);

    /* ------------------ inverse kinematics ------------------ */
    bool ik(Eigen::MatrixXd& tar_position, Eigen::MatrixXd& tar_orientation, double tarFai = 0);
    void cal_ElbowInfo(Eigen::Vector3d& P_s, Eigen::Vector3d& P_w, double Fai, Eigen::Vector3d& P_e,Eigen::Vector3d& P_LJ);

    Eigen::Vector3d cal_ElbowPos(Eigen::Matrix4d& RotMatrix, double Angle, double LinkLen, double Fai);
    Eigen::Matrix4d cal_ElbowRotMatrix(Eigen::Vector3d& P_s, Eigen::Vector3d& P_w, Eigen::Vector3d& E);

    double cal_VecIncAngle(Eigen::Vector3d& v1, Eigen::Vector3d& v2);
    double cal_VecIncAngle(Eigen::Vector3d& origin, Eigen::Vector3d& p1, Eigen::Vector3d& p2);

    /* Normalize Angle between 0 and 360 */
    void normalizeAngle(double& rad);

    double roundN(double num, int digit = 0);
    /* ==================== Evo Kinematics ==================== */
};

}

#endif /* MANIPULATOR_KINEMATICS_DYNAMICS_MANIPULATOR_KINEMATICS_DYNAMICS_H_ */
