/* @author: YueLin */

#include "predictor/bezier.hpp"

namespace diff_track
{
    class Predictor
    {
        private:
            bool empty;
            int* factorial;
            Bezier* bezier;
            Eigen::VectorXd vector;
            Eigen::MatrixXd matrix;
            Eigen::Matrix2Xd queue;
            int m, p, step, observations, stationary;

        public:
            Predictor(Bezier*, double);
            ~Predictor() {delete[] factorial;}

        public:
            void predict();

            void enqueue(double, double);
            
            void reset() {empty = true; p = stationary = 0;}

        private:
            int combination(int n, int k)
            {
                if(k < 0 || k > n) return 0;
                return factorial[n] / (factorial[k] * factorial[n - k]);
            }

            double integral(int i, int j)
            {
                return std::pow(2 * bezier->duration, i + j + 1) * (
                    factorial[i] * factorial[j]
                ) / factorial[i + j + 1];
            }
    };
}