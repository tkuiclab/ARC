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
 * ThorManipulation.cpp
 *
 *  Created on: 2016. 1. 18.
 *      Author: zerom
 */

#include <stdio.h>
#include "manipulator_h_base_module/base_module.h"

using namespace robotis_manipulator_h;

BaseModule::BaseModule()
    : control_cycle_msec_(0)
{
    enable_       = false;
    module_name_  = "base_module";
    control_mode_ = robotis_framework::PositionControl;

    tra_gene_thread_ = NULL;

    result_["joint1"] = new robotis_framework::DynamixelState();
    result_["joint2"] = new robotis_framework::DynamixelState();
    result_["joint3"] = new robotis_framework::DynamixelState();
    result_["joint4"] = new robotis_framework::DynamixelState();
    result_["joint5"] = new robotis_framework::DynamixelState();
    result_["joint6"] = new robotis_framework::DynamixelState();
    result_["joint7"] = new robotis_framework::DynamixelState();


    joint_name_to_id_["joint1"] = 1;
    joint_name_to_id_["joint2"] = 2;
    joint_name_to_id_["joint3"] = 3;
    joint_name_to_id_["joint4"] = 4;
    joint_name_to_id_["joint5"] = 5;
    joint_name_to_id_["joint6"] = 6;
    joint_name_to_id_["joint7"] = 7;

    robotis_     = new RobotisState();
    joint_state_ = new BaseJointState();
    manipulator_ = new ManipulatorKinematicsDynamics(ARM);

    /* init velocity */
    vel_percent  = DEFAULT_SPD;

    for(int i=0;i<=6;i++)
        fk_fb_msg.data.push_back(0.0);
}

BaseModule::~BaseModule()
{
    queue_thread_.join();
}

void BaseModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
    control_cycle_msec_ = control_cycle_msec;
    queue_thread_ = boost::thread(boost::bind(&BaseModule::queueThread, this));
}

void BaseModule::parseIniPoseData(const std::string &path)
{
    YAML::Node doc;
    try
    {
        // load yaml
        doc = YAML::LoadFile(path.c_str());
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Fail to load yaml file.");
        return;
    }

    // parse movement time
    double _mov_time = doc["mov_time"].as<double>();
    robotis_->mov_time_ = _mov_time;

    // parse target pose
    YAML::Node _tar_pose_node = doc["tar_pose"];
    for (YAML::iterator _it = _tar_pose_node.begin(); _it != _tar_pose_node.end(); ++_it)
    {
        int    _id    = _it->first.as<int>();
        double _value = _it->second.as<double>();

        robotis_->joint_ini_pose_.coeffRef(_id, 0) = _value * DEGREE2RADIAN;
    }

    robotis_->all_time_steps_ = int(robotis_->mov_time_ / robotis_->smp_time_) + 1;
    robotis_->calc_joint_tra_.resize(robotis_->all_time_steps_, MAX_JOINT_ID + 1);
}

void BaseModule::queueThread()
{
    ros::NodeHandle    ros_node;
    ros::CallbackQueue callback_queue;

    ros_node.setCallbackQueue(&callback_queue);

    /* publish topics */
    status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
    set_ctrl_module_pub_ = ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);
    FK_FeedBack_pub_     = ros_node.advertise<manipulator_h_base_module_msgs::IK_Cmd>("/robotis/fk_fb", 1);

    /* service */
    ros::ServiceServer get_joint_pose_server = ros_node.advertiseService("/robotis/base/get_joint_pose",
                                                                         &BaseModule::getJointPoseCallback,
                                                                         this);
    ros::ServiceServer get_kinematics_pose_server = ros_node.advertiseService("/robotis/base/get_kinematics_pose",
                                                                         &BaseModule::getKinematicsPoseCallback,
                                                                         this);

    /* subscribe topics */
    ros::Subscriber set_mode_msg_sub = ros_node.subscribe("/robotis/base/set_mode_msg", 5,
                                                          &BaseModule::setModeMsgCallback, this);
    ros::Subscriber ini_pose_msg_sub = ros_node.subscribe("/robotis/base/ini_pose_msg", 5,
                                                          &BaseModule::initPoseMsgCallback, this);
    ros::Subscriber kinematics_pose_msg_sub = ros_node.subscribe("/robotis/base/kinematics_pose_msg", 5,
                                                                 &BaseModule::kinematicsPoseMsgCallback, this);
    
    /* created for arc */
    ros::Subscriber velocity_msg_sub   = ros_node.subscribe("/robotis/base/set_velocity", 5,
                                                            &BaseModule::setVelCallback, this);
    ros::Subscriber joint_pose_msg_sub = ros_node.subscribe("/robotis/base/Joint_Control", 5,
                                                            &BaseModule::JointControlCallback, this);
    ros::Subscriber JointP2P_msg_sub   = ros_node.subscribe("/robotis/base/JointP2P_msg", 5,
                                                            &BaseModule::P2PCallBack, this);
    ros::Subscriber TaskP2P_msg_sub    = ros_node.subscribe("/robotis/base/TaskP2P_msg", 5,
                                                            &BaseModule::LineCallBack, this);

    while (ros_node.ok())
    {
        callback_queue.callAvailable();
        usleep(1000);
    }
}

/* =================================== service =================================== */
bool BaseModule::getJointPoseCallback(manipulator_h_base_module_msgs::GetJointPose::Request &req,
                                      manipulator_h_base_module_msgs::GetJointPose::Response &res)
{
    if (enable_ == false)
        return false;

    for (int id = 1; id <= MAX_JOINT_ID; id++)
    {
        for (int name_index = 0; name_index < req.joint_name.size(); name_index++)
        {
            if (manipulator_->manipulator_link_data_[id]->name_ == req.joint_name[name_index])
            {
                res.joint_name.push_back(manipulator_->manipulator_link_data_[id]->name_);
                // res.joint_value.push_back(joint_state_->curr_joint_state_[id].position_);
                res.joint_value.push_back(joint_state_->goal_joint_state_[id].position_);

                break;
            }
        }
    }

    return true;
}

bool BaseModule::getKinematicsPoseCallback(manipulator_h_base_module_msgs::GetKinematicsPose::Request &req,
                                           manipulator_h_base_module_msgs::GetKinematicsPose::Response &res)
{
    if (enable_ == false)
        return false;

    res.group_pose.position.x = manipulator_->manipulator_link_data_[END_LINK]->position_.coeff(0, 0);
    res.group_pose.position.y = manipulator_->manipulator_link_data_[END_LINK]->position_.coeff(1, 0);
    res.group_pose.position.z = manipulator_->manipulator_link_data_[END_LINK]->position_.coeff(2, 0);

    Eigen::Quaterniond quaternion = robotis_framework::convertRotationToQuaternion(manipulator_->manipulator_link_data_[END_LINK]->orientation_);

    res.group_pose.orientation.w = quaternion.w();
    res.group_pose.orientation.x = quaternion.x();
    res.group_pose.orientation.y = quaternion.y();
    res.group_pose.orientation.z = quaternion.z();
    res.group_redundancy = manipulator_->get_Redundancy();

    return true;
}

/* =================================== subscribe =================================== */
/* ----------------------------------- set mode ----------------------------------- */
void BaseModule::setModeMsgCallback(const std_msgs::String::ConstPtr &msg)
{
    std_msgs::String str_msg;
    str_msg.data = msg->data != "" ? "base_module" : "";

    /* sending empty string can stop all module */
    set_ctrl_module_pub_.publish(str_msg);
    ROS_INFO("set module '%s'", str_msg.data.c_str());
}

/* ----------------------------------- velocity ----------------------------------- */
void BaseModule::setVelCallback(const std_msgs::Float64::ConstPtr &msg)
{
    vel_percent = msg->data * 0.01;
    ROS_INFO("velocity set to %.2f", msg->data);
}

/* ----------------------------------- inital pose ----------------------------------- */
void BaseModule::initPoseMsgCallback(const std_msgs::String::ConstPtr &msg)
{
    if (enable_ == false)
        return;

    if (robotis_->is_moving_ == false)
    {
        if (msg->data == "ini_pose")
        {
            // parse initial pose
            std::string ini_pose_path = ros::package::getPath("manipulator_h_base_module") + "/config/ini_pose.yaml";
            parseIniPoseData(ini_pose_path);

            tra_gene_thread_ = new boost::thread(boost::bind(&BaseModule::generateInitPoseTrajProcess, this));
            delete tra_gene_thread_;
        }
    }
    else
    {
        ROS_INFO("previous task is alive");
    }
}

/* ----------------------------------- joint ----------------------------------- */
void BaseModule::JointControlCallback(const manipulator_h_base_module_msgs::JointPose::ConstPtr &msg)
{
    if (enable_ == false)
        return;

    robotis_->joint_pose_msg_ = *msg;

    if (robotis_->is_moving_ == false)
    {
        tra_gene_thread_ = new boost::thread(boost::bind(&BaseModule::generateJointTrajProcess, this));
        delete tra_gene_thread_;
    }
    else
    {
        ROS_INFO("previous task is alive");
    }
}

/* ----------------------------------- ptp ----------------------------------- *///jmp p2p
void BaseModule::P2PCallBack(const manipulator_h_base_module_msgs::IK_Cmd::ConstPtr &cmd)
{
    if (enable_ == false)
        return;

    /* convert cmd info */
    double x = cmd->data[0];
    double y = cmd->data[1];
    double z = cmd->data[2];

    double roll  = cmd->data[4] * M_PI / 180.0;
    double pitch = cmd->data[3] * M_PI / 180.0;
    double yaw   = cmd->data[5] * M_PI / 180.0;
    robotis_->ik_cmd_fai = cmd->data.size() == 7 ? cmd->data[6] * M_PI / 180.0 : 0.0;

    robotis_->ik_target_position_ << x, y, z;
    // robotis_->ik_target_rotation_ << roll, pitch, yaw;
    robotis_->ik_target_rotation_(0,0) = roll;
    robotis_->ik_target_rotation_(1,0) = pitch;
    robotis_->ik_target_rotation_(2,0) = yaw;
    std::cout<<"ori_data = "<<pitch<<", "<<roll<<", "<<yaw<<"\n";
    std::cout<<"ik_input = "<<robotis_->ik_target_rotation_(0,1)<<", "
                            <<robotis_->ik_target_rotation_(0,0)<<", "
                            <<robotis_->ik_target_rotation_(0,2)<<"\n";
    // robotis_->ik_target_rotation_ = robotis_framework::convertRPYToRotation(roll, pitch, yaw);

    /* calc ik */
    manipulator_->Euler_Mode = e_nsa;
    // manipulator_->Euler_Mode = e_ICLAB;
    bool ik_success = manipulator_->ik(robotis_->ik_target_position_,
                                       robotis_->ik_target_rotation_,
                                       robotis_->ik_cmd_fai);
    // ======================= new ================================
    if(!ik_success)
    {
        bool check_ik_again_success = true;
        double new_fai_Arr[3] = {10, -10, 0};       // for avoid J7_Over180
        double fai_interval   = 10;                 // for avoid Joint Limit
        double new_fai        = 0;
        if(manipulator_->ErrCode.J7_Over180 == true)
        {
            for(int i=0 ; i<=2  ; i++)
            {
                // check_ik_again_success = manipulator_->ik(robotis_->ik_target_position_, robotis_->ik_target_rotation_, new_fai_Arr[i]*M_PI/180);
                if(check_ik_again_success)
                {
                    std::cout<<"new_fai = "<<new_fai_Arr[i]<<"\n";
                    manipulator_->ErrCode.J7_Over180 == false;
                    new_fai = new_fai_Arr[i];
                    break;
                }
                if((i==2)&&(check_ik_again_success==false))
                {
                    std::cout<<"\n====== Neither 10 nor -10 can handle this case!!! ====== \n";
                }
            }
        }
        // if(manipulator_->ErrCode.JointLimit == true)
        // {
        //     for(int i=1 ; i>=-1 ; i-=2 )
        //     {
        //         fai_interval *= (robotis_framework::sign(new_fai)>=0) ? 1 : -1 ;
        //         for(int j = 1 ; j<=9 ; j++)
        //         {
        //             check_ik_again_success = ik(robotis_->ik_target_position_, robotis_->ik_target_rotation_, i*fai_interval[j]*M_PI/180);
        //             if(check_ik_again_success == true)
        //             {
        //                 std::cout<<"Solve joint limit with fai = "<<i*new_fai[j]*M_PI/180<<"\n";
        //                 i = 3;  //for break upper loop
        //                 break;
        //             }
        //         }
        //     }
        // }
        if(manipulator_->ErrCode.JointLimit == true)
            return;
    }
    // ======================= orig ================================
    // if (!ik_success)
    // {
    //     ROS_INFO("PTP: IK ERR !!!");
    //     publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "IK Failed: Joint Limit");
    //     return;
    // }
    // =======================================================

    // manipulator_->fk();
    // std::cout << "FK position_: " << manipulator_->manipulator_link_data_[END_LINK]->position_ << std::endl;
    // std::cout << "FK Redundancy: " << manipulator_->get_Redundancy() * 180 / M_PI << std::endl;

    robotis_->ik_id_start_ = 0;
    robotis_->ik_id_end_   = END_LINK;

    if (robotis_->is_moving_ == false)
    {
        tra_gene_thread_ = new boost::thread(boost::bind(&BaseModule::generateP2PTrajProcess, this));
        delete tra_gene_thread_;
    }
    else
    {
        ROS_INFO("previous task is alive");
    }
}

/* ----------------------------------- line ----------------------------------- */
void BaseModule::LineCallBack(const manipulator_h_base_module_msgs::IK_Cmd::ConstPtr &cmd)
{
    if (enable_ == false)
        return;
    if( fabs(cmd->data[4]) > 89 )
    {
        std::cout<<"[warning] Don't using line fn, when the fabs(roll) is >= 90\n";
        return ;
    }

    /* 記下命令 */
    robotis_->kinematics_pose_msg_.pose.position.x = cmd->data[0];
    robotis_->kinematics_pose_msg_.pose.position.y = cmd->data[1];
    robotis_->kinematics_pose_msg_.pose.position.z = cmd->data[2];
    // Eigen::Quaterniond quaterion = robotis_framework::convertRPYToQuaternion(cmd->data[4] * M_PI / 180.0,
    //                                                                          cmd->data[3] * M_PI / 180.0,
    //                                                                          cmd->data[5] * M_PI / 180.0);
    Eigen::Quaterniond quaterion = robotis_framework::convertEulerToQuaternion(cmd->data[4] * M_PI / 180.0,
                                                                             cmd->data[3] * M_PI / 180.0,
                                                                             cmd->data[5] * M_PI / 180.0);
    
    
    robotis_->kinematics_pose_msg_.pose.orientation.w = quaterion.w();
    robotis_->kinematics_pose_msg_.pose.orientation.x = quaterion.x();
    robotis_->kinematics_pose_msg_.pose.orientation.y = quaterion.y();
    robotis_->kinematics_pose_msg_.pose.orientation.z = quaterion.z();

    // jmp line1
    // std::cout<< "cmd data = " <<cmd->data[3] <<"\n";
    // std::cout<< "cmd data = " <<cmd->data[4] <<"\n";
    // std::cout<< "cmd data = " <<cmd->data[5] <<"\n";
    robotis_->line_ik_pos <<cmd->data[0], cmd->data[1], cmd->data[2];
    robotis_->line_ik_rpy <<cmd->data[4]*M_PI/180.0,
                            cmd->data[3]*M_PI/180.0,
                            cmd->data[5]*M_PI/180.0;
    robotis_->ik_cmd_fai = cmd->data.size() == 7 ? cmd->data[6] * M_PI / 180.0 : 0.0;

    robotis_->ik_id_start_ = 0;
    robotis_->ik_id_end_   = END_LINK;

    if (robotis_->is_moving_ == false)
    {
        tra_gene_thread_ = new boost::thread(boost::bind(&BaseModule::generateTaskTrajProcess, this));
        delete tra_gene_thread_;
    }
    else
    {
        ROS_INFO("previous task is alive");
    }
}

/* ----------------------------------- original line ----------------------------------- */
void BaseModule::kinematicsPoseMsgCallback(const manipulator_h_base_module_msgs::KinematicsPose::ConstPtr &msg)
{
    if (enable_ == false)
        return;

    robotis_->kinematics_pose_msg_ = *msg;

    robotis_->ik_id_start_ = 0;
    robotis_->ik_id_end_   = END_LINK;

    if (robotis_->is_moving_ == false)
    {
        tra_gene_thread_ = new boost::thread(boost::bind(&BaseModule::generateTaskTrajProcess, this));
        delete tra_gene_thread_;
    }
    else
    {
        ROS_INFO("previous task is alive");
    }
}

/* =================================== generate trajectory =================================== */
/* ----------------------------------- inital pose ----------------------------------- */
void BaseModule::generateInitPoseTrajProcess()
{
    if (enable_ == false)
        return;

    for (int id = 1; id <= MAX_JOINT_ID; id++)
    {
        double ini_value = joint_state_->curr_joint_state_[id].position_;
        double tar_value = robotis_->joint_ini_pose_.coeff(id, 0);

        Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                    robotis_->smp_time_, robotis_->mov_time_);

        robotis_->calc_joint_tra_.block(0, id, robotis_->all_time_steps_, 1) = tra;
    }

    robotis_->cnt_ = 0;
    robotis_->is_moving_ = true;

    ROS_INFO("[start] send trajectory");
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");
}

/* ----------------------------------- joint ----------------------------------- */
void BaseModule::generateJointTrajProcess()
{
    if (enable_ == false)
        return;

    /* set movement time */
    double tol      = vel_percent * MAX_JSPD; // rad per sec
    double mov_time = vel_percent * BASE_MOVE_TIME;
    double max_diff = 0.0;

    for (int name_index = 0; name_index < robotis_->joint_pose_msg_.name.size(); name_index++)
    {
        double ini_value = 0;
        double tar_value = 0;

        for (int id = 1; id <= MAX_JOINT_ID; id++)
        {
            if (manipulator_->manipulator_link_data_[id]->name_ == robotis_->joint_pose_msg_.name[name_index])
            {
                ini_value = joint_state_->curr_joint_state_[id].position_;
                tar_value = robotis_->joint_pose_msg_.value[name_index];
                break;
            }
        }

        double abs_diff = fabs(tar_value - ini_value);

        if (max_diff < abs_diff)
            max_diff = abs_diff;
    }

    robotis_->mov_time_ = max_diff / tol;
    int all_time_steps = int(floor((robotis_->mov_time_ / robotis_->smp_time_) + 1.0));
    robotis_->mov_time_ = double(all_time_steps - 1) * robotis_->smp_time_;

    if (robotis_->mov_time_ < mov_time)
        robotis_->mov_time_ = mov_time;

    robotis_->all_time_steps_ = int(robotis_->mov_time_ / robotis_->smp_time_) + 1;
    robotis_->calc_joint_tra_.resize(robotis_->all_time_steps_, MAX_JOINT_ID + 1);

    /* calculate joint trajectory */
    for (int id = 1; id <= MAX_JOINT_ID; id++)
    {
        double ini_value = joint_state_->curr_joint_state_[id].position_;
        double tar_value = ini_value;

        for (int name_index = 0; name_index < robotis_->joint_pose_msg_.name.size(); name_index++)
        {
            if (manipulator_->manipulator_link_data_[id]->name_ == robotis_->joint_pose_msg_.name[name_index])
            {
                tar_value = robotis_->joint_pose_msg_.value[name_index];
                break;
            }
        }

        Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                    robotis_->smp_time_, robotis_->mov_time_);

        robotis_->calc_joint_tra_.block(0, id, robotis_->all_time_steps_, 1) = tra;
    }

    robotis_->cnt_ = 0;
    robotis_->is_moving_ = true;

    ROS_INFO("joint: max_diff: %.3f (deg)", max_diff * RADIAN2DEGREE);
    ROS_INFO("joint: mov_time: %.3f (s)", robotis_->mov_time_);

    ROS_INFO("[start] send trajectory");
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");
}

/* ----------------------------------- ptp ----------------------------------- */
void BaseModule::generateP2PTrajProcess()
{
    if (enable_ == false)
        return;

    /* set movement time */
    double tol      = vel_percent * MAX_JSPD; // rad per sec
    double mov_time = vel_percent * BASE_MOVE_TIME;
    double max_diff = 0.0;

    for (int id = 1; id <= MAX_JOINT_ID; id++)
    {
        double ini_value = joint_state_->curr_joint_state_[id].position_;
        double tar_value = manipulator_->ik_calc_joint_angle_[id - 1];

        double abs_diff = fabs(tar_value - ini_value);

        if (max_diff < abs_diff)
            max_diff = abs_diff;
    }

    robotis_->mov_time_ = max_diff / tol;
    int all_time_steps = int(floor((robotis_->mov_time_ / robotis_->smp_time_) + 1.0));
    robotis_->mov_time_ = double(all_time_steps - 1) * robotis_->smp_time_;

    if (robotis_->mov_time_ < mov_time)
        robotis_->mov_time_ = mov_time;

    robotis_->all_time_steps_ = int(robotis_->mov_time_ / robotis_->smp_time_) + 1;
    robotis_->calc_joint_tra_.resize(robotis_->all_time_steps_, MAX_JOINT_ID + 1);

    /* calculate joint trajectory */
    for (int id = 1; id <= MAX_JOINT_ID; id++)
    {
        double ini_value = joint_state_->curr_joint_state_[id].position_;
        double tar_value = manipulator_->ik_calc_joint_angle_[id - 1];

        Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                    robotis_->smp_time_, robotis_->mov_time_);

        robotis_->calc_joint_tra_.block(0, id, robotis_->all_time_steps_, 1) = tra;
    }

    robotis_->cnt_ = 0;
    robotis_->is_moving_ = true;

    ROS_INFO("ptp: max_diff: %.3f (deg)", max_diff * RADIAN2DEGREE);
    ROS_INFO("ptp: mov_time: %.3f (s)", robotis_->mov_time_);

    ROS_INFO("[start] send trajectory");
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");
}

/* ----------------------------------- line ----------------------------------- */
void BaseModule::generateTaskTrajProcess()
{
    /* set movement time */
    double tol      = vel_percent * MAX_ESPD; // m per sec
    double mov_time = vel_percent * BASE_MOVE_TIME;

    /* end-effoctor */
    double diff = sqrt(
                        pow(manipulator_->manipulator_link_data_[robotis_->ik_id_end_]->position_.coeff(0, 0)
                            - robotis_->kinematics_pose_msg_.pose.position.x, 2)
                      + pow(manipulator_->manipulator_link_data_[robotis_->ik_id_end_]->position_.coeff(1, 0)
                            - robotis_->kinematics_pose_msg_.pose.position.y, 2)
                      + pow(manipulator_->manipulator_link_data_[robotis_->ik_id_end_]->position_.coeff(2, 0)
                            - robotis_->kinematics_pose_msg_.pose.position.z, 2)
                  );

    robotis_->mov_time_ = diff / tol;

    /* redundancy */
    double ini_fai = manipulator_->get_Redundancy();
    double tar_fai = robotis_->ik_cmd_fai;
    double dif_fai = fabs(tar_fai - ini_fai);

    //if (dif_fai >= 0.1)
    {
        robotis_->ik_target_position_ << robotis_->kinematics_pose_msg_.pose.position.x,
                                         robotis_->kinematics_pose_msg_.pose.position.y,
                                         robotis_->kinematics_pose_msg_.pose.position.z;
        robotis_->ik_target_rotation_ = robotis_framework::convertQuaternionToRotation(
                                         Eigen::Quaterniond(
                                         robotis_->kinematics_pose_msg_.pose.orientation.w,
                                         robotis_->kinematics_pose_msg_.pose.orientation.x,
                                         robotis_->kinematics_pose_msg_.pose.orientation.y,
                                         robotis_->kinematics_pose_msg_.pose.orientation.z));

        /* calc ik jmp line2 */
        std::cout   << "line_quat = " 
                    << robotis_->kinematics_pose_msg_.pose.orientation.w <<", "
                    << robotis_->kinematics_pose_msg_.pose.orientation.x <<", "
                    << robotis_->kinematics_pose_msg_.pose.orientation.y <<", "
                    << robotis_->kinematics_pose_msg_.pose.orientation.z <<"\n";

        std::cout   << "Rot Matrix(ik_target_rotation_) = " << robotis_->ik_target_rotation_ <<"\n";

        std::cout<<"line_ik_pos_cmd = " << robotis_->line_ik_pos <<"\n";
        std::cout<<"line_ik_ori_cmd = " << robotis_->line_ik_rpy <<"\n";
        bool ik_success = manipulator_->ik(robotis_->ik_target_position_,
                                           robotis_->line_ik_rpy,
                                           robotis_->ik_cmd_fai);
        if (!ik_success)
        {
            ROS_INFO("LINE: IK WILL ERR !!!");
            publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "IK Failed: Joint Limit");
            return;
        }

        /* set movement time */
        double f_tol      = vel_percent * MAX_JSPD; // rad per sec
        double f_max_diff = 0.0;
        for (int id = 1; id <= MAX_JOINT_ID; id++)
        {
            double ini_value = joint_state_->curr_joint_state_[id].position_;
            double tar_value = manipulator_->ik_calc_joint_angle_[id - 1];
            double abs_diff  = fabs(tar_value - ini_value);

            if (f_max_diff < abs_diff)
                f_max_diff = abs_diff;
        }

        double f_mov_time = f_max_diff / f_tol;
        if (robotis_->mov_time_ < f_mov_time)
            robotis_->mov_time_ = f_mov_time;

        ROS_INFO("line: f diff:   %.3f (deg)", f_max_diff * 180/M_PI);
    }

    /* calculate all step */
    int all_time_steps = int(floor((robotis_->mov_time_ / robotis_->smp_time_) + 1.0));
    robotis_->mov_time_ = double(all_time_steps - 1) * robotis_->smp_time_;

    if (robotis_->mov_time_ < mov_time)
        robotis_->mov_time_ = mov_time;

    robotis_->all_time_steps_ = all_time_steps;//int(robotis_->mov_time_ / robotis_->smp_time_) + 1;
    robotis_->calc_task_tra_.resize(robotis_->all_time_steps_, 3);
 
    ROS_INFO("line: e diff:   %.3f (m)", diff);
    ROS_INFO("line: mov_time: %.3f (s)", robotis_->mov_time_);

    /* calculate trajectory */
    for (int dim = 0; dim < 3; dim++)
    {
        double ini_value = manipulator_->manipulator_link_data_[robotis_->ik_id_end_]->position_.coeff(dim, 0);
        double tar_value;

        if (dim == 0)
            tar_value = robotis_->kinematics_pose_msg_.pose.position.x;
        else if (dim == 1)
            tar_value = robotis_->kinematics_pose_msg_.pose.position.y;
        else if (dim == 2)
            tar_value = robotis_->kinematics_pose_msg_.pose.position.z;

        Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                    robotis_->smp_time_, robotis_->mov_time_);

        robotis_->calc_task_tra_.block(0, dim, robotis_->all_time_steps_, 1) = tra;
    }

    /* generation trajectory fro redundancy */
    Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_fai, 0.0, 0.0, tar_fai, 0.0, 0.0,
                                                               robotis_->smp_time_, robotis_->mov_time_);

    robotis_->calc_fai_tra.resize(robotis_->all_time_steps_, 1);
    robotis_->calc_fai_tra.block(0, 0, robotis_->all_time_steps_, 1) = tra;

    robotis_->cnt_ = 0;
    robotis_->is_moving_ = true;
    robotis_->ik_solve_  = true;

    ROS_INFO("[start] send trajectory");
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Trajectory");
}

/* =================================== main process =================================== */
void BaseModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                         std::map<std::string, double> sensors)
{
    if (enable_ == false)
        return;

    /*----- write curr position -----*/

    for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
         state_iter != result_.end(); state_iter++)
    {
        std::string joint_name = state_iter->first;

        robotis_framework::Dynamixel *dxl = NULL;
        std::map<std::string, robotis_framework::Dynamixel *>::iterator dxl_it = dxls.find(joint_name);
        if (dxl_it != dxls.end())
            dxl = dxl_it->second;
        else
            continue;

        double joint_curr_position = dxl->dxl_state_->present_position_;
        double joint_goal_position = dxl->dxl_state_->goal_position_;

        joint_state_->curr_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_curr_position;
        joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_goal_position;
    }

    /*----- forward kinematics -----*/
    /* 需要下面三行，末端點資訊才會被刷新(教末端點回授) */
    for (int id = 1; id <= MAX_JOINT_ID; id++)
         manipulator_->manipulator_link_data_[id]->joint_angle_ = joint_state_->goal_joint_state_[id].position_;
        //manipulator_->manipulator_link_data_[id]->joint_angle_ = joint_state_->curr_joint_state_[id].position_;

    manipulator_->fk();
    // pub fk info to tyhe topic "/robotis/fk_fb"
    fk_fb_msg.data.clear();
    fk_fb_msg.data.push_back(manipulator_->fk_x);
    fk_fb_msg.data.push_back(manipulator_->fk_y);
    fk_fb_msg.data.push_back(manipulator_->fk_z);
    fk_fb_msg.data.push_back(manipulator_->fk_pitch);
    fk_fb_msg.data.push_back(manipulator_->fk_roll);
    fk_fb_msg.data.push_back(manipulator_->fk_yaw);
    fk_fb_msg.data.push_back(manipulator_->fk_fai);
    FK_FeedBack_pub_.publish(fk_fb_msg);


    /* ----- send trajectory ----- line3*/
    if (robotis_->is_moving_ == true)//ptp line
    {
        if (robotis_->cnt_ == 0)
        {
            robotis_->ik_start_rotation_ = manipulator_->manipulator_link_data_[robotis_->ik_id_end_]->orientation_;
            
            // convert error rot matrix to rpy and then fix it
            Eigen::MatrixXd tmp_start_pry = robotis_framework::convertRotationToRPY(robotis_->ik_start_rotation_);
            tmp_start_pry(2) += 90*M_PI/180.0;
            // convert the correct rpy to quaternion and then convert back to rotation matrix
            Eigen::Quaterniond start_quaternion = robotis_framework::convertEulerToQuaternion( 
                                                                            tmp_start_pry(2),
                                                                            tmp_start_pry(0),
                                                                            tmp_start_pry(1));
            Eigen::MatrixXd tmp_rot = robotis_framework::convertQuat2Rotation(start_quaternion);
            for(int i=0 ; i<=2;i++)
            {
                for(int j=0 ; j<=2 ; j++)
                {
                    robotis_->ik_start_rotation_(i,j) = tmp_rot(i,j);
                }
            }
        }
        if (robotis_->ik_solve_ == true) //line
        {
            robotis_->setInverseKinematics();

            /* kinematics of evo  */
            std::cout<<"process"<<"\n";
            // 
            std::cout<<"ik_curr_pos = " << robotis_->ik_target_position_ <<"\n";
            robotis_->line_ik_rpy <<robotis_->line_ik_rpy(0,0), 
                                    robotis_->line_ik_rpy(1,0), 
                                    robotis_->line_ik_rpy(2,0);
            // std::cout<<"ik_curr_ori00 = " << robotis_->line_ik_rpy(0,0) <<"\n";
            // std::cout<<"ik_curr_ori01 = " << robotis_->line_ik_rpy(0,1) <<"\n";
            // std::cout<<"ik_curr_ori02 = " << robotis_->line_ik_rpy(0,2) <<"\n";
            // std::cout<<"ik_curr_ori00 = " << robotis_->line_ik_rpy(0,0) <<"\n";
            // std::cout<<"ik_curr_ori10 = " << robotis_->line_ik_rpy(1,0) <<"\n";
            // std::cout<<"ik_curr_ori20 = " << robotis_->line_ik_rpy(2,0) <<"\n";
            std::cout<<"ik_curr_ori20 = " << robotis_->line_ik_rpy<<"\n";

            bool ik_success = manipulator_->ik(robotis_->ik_target_position_, robotis_->line_ik_rpy, robotis_->ik_target_fai);
            if (ik_success == true)
            {
                for (int id = 1; id <= MAX_JOINT_ID; id++)
                    joint_state_->goal_joint_state_[id].position_ = manipulator_->manipulator_link_data_[id]->joint_angle_;
            }
            else
            {
                ROS_INFO("[end] send trajectory (ik failed)");
                publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "End Trajectory (IK Failed)");

                robotis_->is_moving_ = false;
                robotis_->ik_solve_ = false;
                robotis_->cnt_ = 0;
            }
        }
        else //ptp
        {
            for (int id = 1; id <= MAX_JOINT_ID; id++)
            {
                joint_state_->goal_joint_state_[id].position_ = robotis_->calc_joint_tra_(robotis_->cnt_, id);
                // std::cout<<"Joint " <<id <<" = " <<joint_state_->goal_joint_state_[id].position_;
            }
        }

        robotis_->cnt_++;
    }

    /*----- set joint data -----*/
    for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
         state_iter != result_.end(); state_iter++)
    {
        std::string joint_name = state_iter->first;
        result_[joint_name]->goal_position_ = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_;
    }

    /*---------- initialize count number ----------*/
    if (robotis_->cnt_ >= robotis_->all_time_steps_ && robotis_->is_moving_ == true)
    {
        ROS_INFO("[ end ] send trajectory");
        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "End Trajectory");

        robotis_->is_moving_ = false;
        robotis_->ik_solve_ = false;
        robotis_->cnt_ = 0;
    }
}

void BaseModule::stop()
{
    robotis_->is_moving_ = false;
    robotis_->ik_solve_ = false;
    robotis_->cnt_ = 0;

    return;
}

bool BaseModule::isRunning()
{
    return robotis_->is_moving_;
}

void BaseModule::publishStatusMsg(unsigned int type, std::string msg)
{
    robotis_controller_msgs::StatusMsg status;
    status.header.stamp = ros::Time::now();
    status.type        = type;
    status.module_name = "Base";
    status.status_msg  = msg;

    status_msg_pub_.publish(status);
}
