/* @author: YueLin */

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "simulator/camera.hpp"

namespace sim
{
    Camera::Camera(std::string robot, Robot& r, double alpha, double distance): 
        angle(alpha), range(distance) 
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.pose.position.x = r.radius * 2;
        ps.pose.position.y = 0;
        fov->poses.push_back(ps);
        fov->header.frame_id = robot;
        ps.header.frame_id = fov->header.frame_id;
        fov[1].header.frame_id = fov->header.frame_id;
        for(double theta = -angle; theta <= angle; theta += deg2rad)
        {
            ps.pose.position.x = distance * std::cos(theta) + r.radius * 2;
            ps.pose.position.y = distance * std::sin(theta);
            fov->poses.push_back(ps);
        }
        fov->poses.push_back(fov->poses.front());

        gaussian = std::normal_distribution<double>(0, 0.01);
    }

    void Camera::update(RandomMap* map, Robot& robot, Robot& obstacle)
    {
        cloud.clear();
        const double r2 = obstacle.radius * obstacle.radius;
        const double w = map->width / 2, h = map->height / 2;
        for(double theta = -angle; theta <= angle; theta += deg2rad)
        {
            double d = map->resolution;
            double yaw = robot.pose.yaw + theta;
            double sin = std::sin(yaw), cos = std::cos(yaw);
            while(d <= range)
            {
                double x = robot.pose.x + d * cos;
                double y = robot.pose.y + d * sin;
                if(map->cost(x, y) <= 0 ||
                   std::fabs(x) >= w || std::fabs(y) >= h ||  
                   hypot2(x - obstacle.pose.x, y - obstacle.pose.y) <= r2)
                {
                    cloud.emplace_back(x + noise(), y + noise(), 0); break;
                }
                d += map->resolution;
            }
        }
    }
}
