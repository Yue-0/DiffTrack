/* @author: YueLin */

#include "predictor/bezier.hpp"

namespace diff_track
{
    Bezier::Bezier(int num, double t)
    : n(num - 1), time(t), duration(t * (num >> 1))
    {
        /* Initialize control points */
        control.resize(2, num);

        /* Precompute factorials */
        factorial = new int[num];
        for(int i = *factorial = 1; i < num; i++)
            factorial[i] = i * factorial[i - 1];
    }

    Eigen::Vector2d Bezier::get(double t)
    {
        Eigen::Vector2d point(0, 0);
        t = std::max(std::min(t, duration), -duration);
        for(int p = 0; p <= n; p++)
            point += control.col(p) * bernstein(t, p);
        return point;
    }

    Eigen::Matrix2Xd Bezier::trajectory()
    {
        int num = n / 2 + 1;
        Eigen::Matrix2Xd path(2, num);
        for(int p = 0; p < num; p++)
            path.col(p) = get(time * p);
        return path;
    }

    Eigen::Vector2d Bezier::derivative(double t)
    {
        Eigen::Vector2d point(0, 0);
        t = std::max(std::min(t, duration), -duration);
        for(int p = 0; p <= n; p++)
            point += control.col(p) * (
                bernstein(t, p - 1, n - 1) - bernstein(t, p, n - 1)
            );
        return point * (n / (duration * 2));
    }
}