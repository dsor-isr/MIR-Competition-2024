/*
Developers: João Lehodey - joao.lehodey@tecnico.ulisboa.pt - DSOR/ISR team (Instituto Superior Tecnico) 
*/

#include "glassy_utils/GlassyGeneralUtils.h"


// Angle conversions
template <typename T>
double deg2rad(T degrees)
{
    return double(degrees) * M_PI / 180.0;
}

template <typename T>
double rad2deg(T radians)
{
    return double(radians) * 180.0 / M_PI;
}


// Quaternion conversions
Eigen::Vector3d quat_to_euler_ZYX(Eigen::Quaterniond q){
    // q = [w, x, y, z] 
    Eigen::Vector3d euler;

    // use try catch just to ensure that the function does not crash
    try
    {
        // EQUATIONS TAKEN FROM: Appendix B.3 -> Small Unmanned Aircrafts: Theory and Practice (small simplification, seen on wikipedia) (use the fact that they are unit quaternions (sum of squares = 1))
        euler[0] = atan2(2*(q.w()*q.x() + q.y()*q.z()), 1 - 2*(q.x()*q.x() + q.y()*q.y())); //roll
        euler[1] = asin(2*(q.w()*q.y() - q.x()*q.z())); // pitch
        euler[2] = atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(q.y()*q.y() + q.z()*q.z())); // yaw
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return euler;
}


