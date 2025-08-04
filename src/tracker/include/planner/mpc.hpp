/* @author: YueLin */

#include "planner/map.hpp"
#include "predictor/bezier.hpp"

namespace diff_track
{
    class ModelPredictiveControl
    {
        private:
            Map* map;
            Bezier* bezier;
        
            /* Hyperparameters */
            int n;
            double lambda[6];
            double radius, ob, dt, kappa;
            
            /* Motion constraints */
            double vel2, acc2, omega2, alpha2;

            /* For L-BFGS optimization */
            int past, mem;
            Eigen::MatrixXd dx, dg;
            Eigen::VectorXd x, g, x0, g0;
            Eigen::VectorXd pf, limit, memory;
            double eps, steps, delta, epsilon;

            /* For Lewis-Overton line search */
            int iterations;
            double wolfe, armijo;
            const double o = 1e-16;

            /* Cost values */
            double time;
            Eigen::VectorXd costs;
            enum Cost {A = 0, D = 1, O = 2, C = 3, F = 4, S = 5};

            /* States */
            Eigen::VectorXd sin, cos;
            Eigen::MatrixXd trajectory;
            Eigen::Matrix2Xd vel, acc;
            Eigen::Matrix3Xd pos;

            /* For gradients propagation */
            Eigen::MatrixXd gx2v, gy2v, gx2w, gy2w, gz2w;
            Eigen::Matrix3Xd gradient;
        
        public:
            ModelPredictiveControl(
                /* Map and Bezier curve */
                Map*, Bezier*,

                /* Robot settings */
                double, double, double,

                /* Motion constraints */
                double, double, double, double,

                /* Weights of cost terms */
                double, double, double, double, double, double,

                /* Hyperparameters for L-BFGS */
                int, int, int, double, double, double, double,

                /* Hyperparameters for Lewis-Overton line search */
                int, double, double
            );

        public:
            void clock(const double t) {time = t;}

            Eigen::MatrixXd control(const Eigen::VectorXd&, 
                                    const Eigen::MatrixXd&);

        private:
            /* Cost function */
            inline double cost();

            /* State propagation */
            inline void propagate();

            /* L-BFGS optimizer */
            inline double optimize();

            /* Convergance test */
            inline bool convergance() const;

            /* Lewis-Overton line search */
            inline bool search(double*, double*, const Eigen::VectorXd&);

            /* Tracking distance cost function */
            inline double td(const Eigen::Vector2d&, Eigen::Vector2d&) const;
    };
}
