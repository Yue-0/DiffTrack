/* @author: YueLin */

#include <random>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "simulator/map.hpp"
#include "simulator/robot.hpp"

namespace sim
{
    class Camera
    {
        private:
            double angle, range;
            nav_msgs::msg::Path fov[2];
            const double deg2rad = PI / 180;
            std::default_random_engine random;
            pcl::PointCloud<pcl::PointXYZ> cloud;
            std::normal_distribution<double> gaussian;
        
        public:
            Camera(std::string, Robot&, double, double);
        
        public:
            void update(RandomMap*, Robot&, Robot&);
            nav_msgs::msg::Path* view() {return fov;}
            nav_msgs::msg::Path* empty() {return fov + 1;}
            pcl::PointCloud<pcl::PointXYZ>* depth() {return &cloud;}
        
        private:
            double noise() {return gaussian(random);}
    };
}
