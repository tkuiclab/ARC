#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <Eigen/Eigen>

void kinematicsCallback(const std_msgs::Float64MultiArray::ConstPtr& cmd);
void Gen_DHLinksTable();
void Gen_TFMat(int index, double Theta, Eigen::Matrix4d& A);
void ForwardKinematics(std::vector<double>& angle, std::vector<double>& pose);
void InverseKinematics(std::vector<double>& pose, std::vector<double>& angle);

const int DOF = 7;
/* DHLinksTable */
Eigen::MatrixXd DH(7, 4);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "evo_kinematics");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("kinematics_cmd", 50, kinematicsCallback);

    Gen_DHLinksTable();

    ros::spin();
    return 0;
}

void kinematicsCallback(const std_msgs::Float64MultiArray::ConstPtr& cmd)
{
    /* end-effector position and euler */
    std::vector<double> pose(6);

    std::cout << "Desired Cmd:" << std::endl;
    for (int i = 0; i < cmd->data.size(); i++)
    {
        pose[i] = cmd->data[i];
        std::cout << pose[i] << " ";
    }
    std::cout << std::endl;

    std::vector<double> angle(7);
    InverseKinematics(pose, angle);

    std::cout << "Calc Angle:" << std::endl;
    std::vector<double> _angle(7);
    for (int i = 0; i < angle.size(); i++)
    {
        _angle[i] = angle[i] * 180.0 / M_PI;
        std::cout << _angle[i] << " ";
    }
    std::cout << std::endl;

    std::vector<double> _pose(6);
    ForwardKinematics(_angle, _pose);

    std::cout << "Calc Cmd:" << std::endl;
    for (int i = 0; i < _pose.size(); i++)
    {
        if (i > 2)
            _pose[i] = _pose[i] * 180.0 / M_PI;
        std::cout << _pose[i] << " ";
    }
    std::cout << std::endl;
}

void ForwardKinematics(std::vector<double>& angle, std::vector<double>& pose)
{
    for (int i = 0; i < angle.size(); i++)
        angle[i] = angle[i] * M_PI / 180.0;
        
    //Eigen::Vector3d JointPos;
    //Eigen::Matrix3d JointDir;
    Eigen::Matrix4d T0_6 = Eigen::Matrix4d::Identity();

    for (int i = 0; i < angle.size(); i++)
    {
        Eigen::Matrix4d A;
        Gen_TFMat(i, angle[i], A);

        T0_6 *= A;
        //JointPos = [Info.JointPos, Info.T0_6(1:3, 4)  ];
        //JointDir = [Info.JointDir, Info.T0_6(1:3, 1:3)];
    }
    Eigen::Vector3d pos = T0_6.block<3, 1>(0, 3);
    Eigen::Matrix3d ori = T0_6.block<3, 3>(0, 0);

/* --------------------------------------------- position -------------------------------------------- */
    pose[0] = pos(0);   // x
    pose[1] = pos(1);   // y
    pose[2] = pos(2);   // z

/* ------------------------------------------- orientation ------------------------------------------- */
    pose[3] = atan2(ori(2, 2), sqrt(1 - pow(ori(2, 2), 2)));  // pitch

    if (abs(ori(2, 2)) < 0.9999)
    {
        double Cz =  ori(1, 2) / sqrt(1 - pow(ori(2, 2), 2));
        double Sz = -ori(0, 2) / sqrt(1 - pow(ori(2, 2), 2));
        pose[5] = atan2(Sz, Cz);    // yaw

        double Cy =  ori(2, 0) / sqrt(1 - pow(ori(2, 2), 2));
        double Sy = -ori(2, 1) / sqrt(1 - pow(ori(2, 2), 2));
        pose[4] = atan2(Sy, Cy);    // roll
        //Info.Rot  = 0;
    }
    else
    {
        pose[5] = angle[0];    // yaw

        pose[4] = (atan2(ori(1, 1), ori(0, 1)) - pose[5]);  // roll
        pose[4] = pose[3] > 0? pose[4]: -pose[4];

        if (pose[4] > M_PI)
            pose[4] -= M_2_PI;
        else if (pose[4] < -M_PI)
            pose[4] += M_2_PI;
        //Info.Rot =  atan2( R0_6(2,2), R0_6(1,2) );
    }
}

void InverseKinematics(std::vector<double>& pose, std::vector<double>& angle)
{
    double x = pose[0];
    double y = pose[1];
    double z = pose[2];
    double pitch = pose[3] * M_PI / 180.0;
    double roll  = pose[4] * M_PI / 180.0;
    double yaw   = pose[5] * M_PI / 180.0;
    
    double Cx = cos(pitch);
    double Sx = sin(pitch);
    double Cy = cos(roll);
    double Sy = sin(roll);
    double Cz = cos(yaw);
    double Sz = sin(yaw);
    
    Eigen::Vector3d position(x, y, z);          // desired position
    Eigen::Matrix3d RPY_Rot;                    // desired orientation
    RPY_Rot << Cz*Sy + Sz*Sx*Cy,  Cz*Cy - Sz*Sx*Sy, -Sz*Cx,
               Sz*Sy - Cz*Sx*Cy,  Sz*Cy + Cz*Sx*Sy,  Cz*Cx,
               Cx*Cy,            -Cx*Sy,             Sx;
    
    int Elbow = -1;                                                 // Elbow Up = -1, Elbow Down =  1
    int Wrist = -1;                                                 // Wrist Up =  1, Wrist Down = -1
    
    double L1 = DH(0, 2);
    double L3 = DH(2, 0);
    double L4 = DH(4, 2);
    double L5 = DH(6, 2);
    double L2 = sqrt(pow(DH(2, 2), 2) + pow(L3, 2));
    double F  = DH(2, 2) + DH(4, 2);

    /* --------------------------------------------- position -------------------------------------------- */
    Eigen::Vector3d hand_pose(L5 * RPY_Rot(0, 2), L5 * RPY_Rot(1, 2), L5 * RPY_Rot(2, 2));
    Eigen::Vector3d WristPos = position - hand_pose;

    if (abs(WristPos(0)) < 0.0001 && abs(WristPos(1)) < 0.0001)
        angle[0] = 0.12345;
    else
        angle[0] = atan2(-WristPos(0) , WristPos(1));

    double xc_2 = pow(WristPos(0), 2);
    double yc_2 = pow(WristPos(1), 2);

    double D = (WristPos - Eigen::Vector3d(0, 0, L1)).norm();
    double E = (WristPos - Eigen::Vector3d(0, 0, 0)).norm();

    double L34 = sqrt(pow(L3, 2) + pow(L4, 2));

    double Alpha = acos((pow(L2, 2) + pow(D, 2) - pow(L34, 2)) / (2 * L2 * D));
    double Gamma = acos((pow(L1, 2) + pow(D, 2) - pow(E, 2))   / (2 * L1 * D));
    double Beta  = 81.25 * M_PI / 180.0;
    double Lunda = 8.75  * M_PI / 180.0;
    
    double Epslon = acos( (pow(L2, 2) + pow(L34, 2) - pow(D, 2)) / (2 * L2 * L34));
    double Fai    = acos( (pow(L2, 2) + pow(L34, 2) - pow(F, 2)) / (2 * L2 * L34));

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

    angle[5] = atan2(Wrist * sqrt(1 - pow(R4_6(2, 2), 2)), R4_6(2, 2));
    
    if (abs(R4_6(2, 2)) > 0.9999)
    {
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
}

void Gen_DHLinksTable()
{
    /* a alpha d delta */
    DH << 0,  M_PI_2,   16, M_PI_2,
          0,  M_PI_2,    0, M_PI_2,
          3, -M_PI_2,   27, M_PI,
          3, -M_PI_2,    0, M_PI,
          0, -M_PI_2, 25.5,    0,
          0,  M_PI_2,    0,    0,
          0,       0, 22.5,    0;

    //std::cout << DH << std::endl;
}

void Gen_TFMat(int index, double theta, Eigen::Matrix4d& A)
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