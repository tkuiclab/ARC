#ifdef _EVO_KINEMATICS_H_
#define _EVO_KINEMATICS_H_

#include <vector>

namespace iclab
{

struct PoseInfo
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;

    inline PoseInfo(): x(0), y(0), z(0), roll(0), pitch(0), yaw(0) {}
    inline PoseInfo(double x, double y, double z): 
        x(x), y(x), z(x), roll(0), pitch(0), yaw(0) {}
    inline PoseInfo(double x, double y, double z, double roll, double pitch, double yaw):
        x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) {}
}

class CKinematics
{
    Eigen::MatrixXd DH;

public:i
    CKinematics() { DH(7, 4); }
    void set_DHLinks(int link, double a, double d, double alpha, double beta);
    
};

}
