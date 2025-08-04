/* @author: YueLin */

#include "predictor/predictor.hpp"

namespace diff_track
{
    Predictor::Predictor(Bezier* b, double hz): 
        bezier(b), m(b->n >> 1), step(b->time * hz)
    {
        /* Initialize variables */
        int n = b->n;
        observations = 1 + std::round(b->duration * hz);
        queue = Eigen::Matrix2Xd::Zero(2, observations);

        /* Initialize factorial */
        int c = n * 2 - 2;
        factorial = new int[c];
        for(int i = *factorial = 1; i < c; i++)
            factorial[i] = i * factorial[i - 1];

        /* Initialize matrix and vector */
        Eigen::MatrixXd a = Eigen::MatrixXd::Zero((n / 2 + 1) * 2, 2 * (1 + n));
        c = a.rows() + (n + 1) * 2;
        vector = Eigen::VectorXd::Zero(c);
        matrix = Eigen::MatrixXd::Zero(c, c);

        /* Calculate matrix */
        for(int i = 0; i <= n; i++)
            for(int j = 0; j <= n; j++)
            {
                c = combination(n - 2, i) * combination(n - 2, j);
                if(c) matrix(i, j) += c * integral(i + j, 2 * n - i - j - 4);

                c = combination(n - 2, i - 2) * combination(n - 2, j - 2);
                if(c) matrix(i, j) += c * integral(i + j - 4, 2 * n - i - j);

                c = combination(n - 2, i) * combination(n - 2, j - 2)
                  + combination(n - 2, i - 2) * combination(n - 2, j)
                  + combination(n - 2, i - 1) * combination(n - 2, j - 1) * 4;
                if(c) matrix(i, j) += c * integral(
                    i + j - 2, 2 * n - i - j - 2
                );

                c = combination(n - 2, i) * combination(n - 2, j - 1)
                  + combination(n - 2, i - 1) * combination(n - 2, j);
                if(c) matrix(i, j) -= 2 * c * integral(
                    i + j - 1, 2 * n - i - j - 3
                );
                
                c = combination(n - 2, i - 1) * combination(n - 2, j - 2)
                  + combination(n - 2, i - 2) * combination(n - 2, j - 1);
                if(c) matrix(i, j) -= 2 * c * integral(
                    i + j - 3, 2 * n - i - j - 1
                );
            }
        c = n + 1;
        matrix.block(0, 0, c, c) *= n * n * (n - 1) * (n - 1) / std::pow(
            2 * b->duration, n << 1
        );
        matrix.block(c, c, c, c) = matrix.block(0, 0, c, c);
        
        /* Constraints */
        n = n / 2 + 1;
        for(int i = 0; i < n; i++)
            for(int j = 0; j < c; j++)
                a(i, j) = b->bernstein(i * b->time - b->duration, j);
        a.block(n, c, n, c) = a.block(0, 0, n, c);
        matrix.block(c << 1, 0, n << 1, c << 1) = a;
        matrix.block(0, c << 1, c << 1, n << 1) = a.transpose();

        /* Calculate the inverse matrix */
        matrix = matrix.inverse().eval().topRows(c << 1);
    }

    void Predictor::predict()
    {
        /* Get observation vector */
        for(int i = m, t = observations; i >= 0; t -= (i--, step))
        {
            vector[i + m * 4 + 2] = queue(0, (t + p) % observations);
            vector[i + m * 5 + 3] = queue(1, (t + p) % observations);
        }

        /* Solve KKT point */
        Eigen::VectorXd kkt = matrix * vector;

        /* Set control points */
        bezier->control.row(0) = kkt.head(m * 2 + 1).transpose();
        bezier->control.row(1) = kkt.tail(m * 2 + 1).transpose();
    }

    void Predictor::enqueue(double x, double y)
    {
        if(empty)
        {
            p = 0;
            empty = false;
            stationary = 3;
        }
        else
        {
            if((queue.col(p) - Eigen::Vector2d(x, y)).squaredNorm() > 1e-3)
                stationary = 0;
            else
                stationary = std::min(stationary + 1, 3);
            p = (p + 1) % observations;
        }
        if(stationary == 3)
        {
            queue.row(0).setConstant(x);
            queue.row(1).setConstant(y);
        }
        else
        {
            queue(0, p) = x; queue(1, p) = y;
        }
    }
}
