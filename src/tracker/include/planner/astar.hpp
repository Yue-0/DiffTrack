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
            Eigen::Vector2d trigon;
            Eigen::MatrixXd states;
            const double inf = std::numeric_limits<double>::infinity();

        public:
            ~HybridAStar();
            HybridAStar(Map*, Bezier*, int, int,
                        double, double, double, double, double, double, double);

        public:
            bool plan(Eigen::MatrixXd&, const Eigen::VectorXd&);

        private:
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
            
            bool occlusion(const Eigen::Vector2d&, const Eigen::Vector2d&);

            inline bool visible(const Eigen::Vector2d&, const Eigen::Vector3d&);
    };
}
