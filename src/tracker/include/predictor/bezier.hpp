/* @author: YueLin */

#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Eigen>

namespace diff_track
{
    class Bezier
    {
        public:
            int n;
            int* factorial;
            double time, duration;
            Eigen::Matrix2Xd control;

        public:
            Bezier(int, double);
            ~Bezier() {delete[] factorial;}
            
            Eigen::Vector2d get(double);         // b(t)
            Eigen::Matrix2Xd trajectory();       // b(0) ~ b(duration)
            Eigen::Vector2d derivative(double);  // b'(t)

            /* Bernstein function */
            double bernstein(double t, int i, int m)
            {
                if(i < 0 || i > m) return 0;
                return factorial[m] / (factorial[i] * factorial[m - i])
                     * std::pow(duration - t, m - i)
                     * std::pow(duration + t, i)
                     / std::pow(2 * duration, m)
                     ;
            }
            double bernstein(double t, int i) {return bernstein(t, i, n);}
    };
}