/*
Developers: João Lehodey - joao.lehodey@tecnico.ulisboa.pt - DSOR/ISR team (Instituto Superior Tecnico) 
*/

#ifndef _GlassyGeneralUtils_
#define _GlassyGeneralUtils_

#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>


// Angle conversions
template <typename T>
T deg2rad(T degrees);

template <typename T>
T rad2deg(T radians);

Eigen::Vector3d quat_to_euler_ZYX(Eigen::Quaterniond q);

#endif