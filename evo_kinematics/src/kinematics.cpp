#include "kinematics.h"

void iclab::CKinematics::set_DHLinks(int link, double a, double d, double alpha, double beta)
{
    DH << a, d, alpha, beta;

}
