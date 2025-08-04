/* @author: YueLin */

#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "simulator/utils.hpp"

namespace sim
{
    struct SE2
    {
        double x, y, yaw;

        SE2(): x(0), y(0), yaw(0) {}

        SE2(double x0, double y0, double theta):
            x(x0), y(y0), yaw(theta) {}

        SE2(geometry_msgs::msg::Twist v):
            x(v.linear.x), y(v.linear.y), yaw(v.angular.z) {}
    };

    class Robot
    {
        public:
            SE2 pose, vel;
            double radius;
        
        private:
            nav_msgs::msg::Path path;

        public:
            Robot(double, double, double, double);
        
        public:
            void move(double);

            void control(SE2 v) {vel = v;}

            void transform(
                const std::string&,
                tf2_ros::TransformBroadcaster*,
                geometry_msgs::msg::TransformStamped&
            );

            nav_msgs::msg::Odometry odom(const std::string&,
                                         const std::string&);

            nav_msgs::msg::Path* trajectory(const std::string&);
    };
}
