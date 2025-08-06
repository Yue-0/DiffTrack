/* @author: YueLin */

#pragma once

#include <cmath>
#include <limits>

const double PI = std::acos(-1);
const double INF = std::numeric_limits<double>::infinity();

inline double normalize(double rad)
{
    if(rad > PI || rad <= -PI)
        return 2 * std::floor(0.5 - rad / (2 * PI)) * PI + rad;
    return rad;
}

inline double hypot2(double x, double y)
{
    return x * x + y * y;
}

inline double quaternion2yaw(double qx, double qy, double qz, double qw)
{
    double x = qx * qx, y = qy * qy, z = qz * qz, w = qw * qw;
    if(std::fabs(x + y + z + w) < 1e-6)
        return INF;
    double s = 2 * (qw * qy - qx * qz) / (x + y + z + w);
    if(std::fabs(s) >= 1 - 1e-6) 
        return normalize(std::copysign(2 * std::atan2(qy, qx), s));
    return std::atan2(2 * (qx * qy + qz * qw), w + x - y - z);
}
