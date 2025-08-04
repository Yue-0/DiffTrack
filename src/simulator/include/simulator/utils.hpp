/* @author: YueLin */

#pragma once

#include <cmath>

const double PI = std::acos(-1);

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
