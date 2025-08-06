/* @author: YueLin */

#include <vector>

#include <Eigen/Eigen>

#include "simulator/map.hpp"     
#include "simulator/robot.hpp"

namespace sim
{
    class Planner
    {
        private:
            Robot* robot;
            RandomMap* map;
            double time, ds, vm, wm, vm2, am2, sample, orientation;

            /* ROS message */
            nav_msgs::msg::Path msg;

            /* For A* algorithm */
            double* g;
            int* parent;
            bool* closed;
            int rows, cols;

            /* For B-spline parameterization */
            Eigen::Vector3d dp, dv, da;
            const double SQR2 = std::sqrt(2);

            /* For B-spline optimization */
            const double LAMBDA = 1e4;

            /* For L-BFGS optimization */
            const int PAST = 3;
            const int MEMORY = 8;
            const double EPS = 1e-6;
            const double STEP = 1e20;
            const double DELTA = 1e-6;
            const double EPSILON = 1e-5;
            Eigen::VectorXd pf, limit, memory;

            /* For Lewis-Overton line search */
            const int ITERATIONS = 64;
            const double WOLFE = 9e-1;
            const double ARMIJO = 1e-4;
        
        public:
            ~Planner();
            Planner(Robot*, RandomMap*, double, double, double);

        public:
            geometry_msgs::msg::Twist control();
            
            nav_msgs::msg::Path* message(const std::string& frame) 
            {
                msg.header.frame_id = frame; return &msg;
            }

            void plan(const geometry_msgs::msg::Pose&);

        private:
            /* For path searching */

            cv::Point bfs(const cv::Point&);

            inline cv::Point decode(int) const;

            inline int encode(const cv::Point&) const;

            inline cv::Point tf(double, double) const;

            inline Eigen::Vector2d tf(const cv::Point&) const;

            std::vector<int> dijkstra(const Eigen::MatrixXd&);

            std::vector<cv::Point> astar(cv::Point, cv::Point);

            std::vector<cv::Point> expand(const cv::Point&) const;

            Eigen::Matrix2Xd search(const cv::Point&, const cv::Point&);

            inline double f(int, const cv::Point&, const cv::Point&) const;

            /* For B-spline optimization */

            Eigen::Matrix2Xd bspline(const Eigen::Matrix2Xd&);

            double cost(const Eigen::VectorXd&, Eigen::VectorXd&);

            /* For L-BFGS solver */

            double optimize(Eigen::Matrix2Xd&);

            bool search(Eigen::VectorXd&, double*, 
                        Eigen::VectorXd&, double*, 
                        const Eigen::VectorXd&,
                        const Eigen::VectorXd&,
                        const Eigen::VectorXd&);
            
            inline bool convergance(const Eigen::VectorXd&,
                                    const Eigen::VectorXd&) const;

            /* Convert to ROS message */
            void message(Eigen::Matrix2Xd);
    };
}
