/* @author: YueLin */

#include <limits>

#include "planner/map.hpp"
#include "predictor/bezier.hpp"

namespace diff_track
{
    class HybridAStar
    {
        private:
            Map* map;
            Bezier* bezier;
            int size, angles;
            double radius, dt, vm, wm, dv, dw, ob, epsilon, duration;
        
        private:
            double* g;
            int* parent;
            bool* closed;
            double* time;
            int reacquisition, len;
            Eigen::Vector2d trigon;
            Eigen::MatrixXd states;
            std::vector<cv::Point> points;
            const double sqr = std::sqrt(2);
            const double inf = std::numeric_limits<double>::infinity();
            Eigen::Vector2d nan = Eigen::Vector2d::Constant(
                std::numeric_limits<double>::quiet_NaN()
            );

        public:
            ~HybridAStar();
            HybridAStar(Map*, Bezier*, int, int, 
                        double, double, double, double, 
                        double, double, double, double);

        public:
            bool plan(Eigen::MatrixXd&, const Eigen::VectorXd&);
            Eigen::MatrixXd reacquire(bool, const Eigen::VectorXd&);

        private:
            int dijkstra(const cv::Point&);

            inline void trigonometric(double);

            inline int hash(const Eigen::Vector3d&);

            inline double f(double, 
                            const Eigen::Vector3d&, 
                            const Eigen::Vector3d&);

            Eigen::Vector2d bfs(const Eigen::Vector2d&);

            Eigen::MatrixXd search(Eigen::VectorXd,
                                   const Eigen::Vector2d&,
                                   const Eigen::Vector3d&);

            Eigen::Vector3d proposal(const Eigen::Vector2d&,
                                     const Eigen::Vector2d&);

            std::vector<Eigen::VectorXd> escape(Eigen::VectorXd);

            std::vector<Eigen::VectorXd> expand(const Eigen::VectorXd&);
            
            bool occlusion(const Eigen::Vector2d&, const Eigen::Vector2d&);

            std::vector<cv::Point> expand(const cv::Size&, const cv::Point&);

            inline bool arrival(const Eigen::Vector2d&, const Eigen::Vector2d&);

            inline bool visible(const Eigen::Vector2d&, const Eigen::Vector3d&);
    };
}
