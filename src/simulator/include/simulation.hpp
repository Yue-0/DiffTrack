/* @author: YueLin */

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "simulator/env.hpp"
#include "simulator/camera.hpp"
#include "simulator/planner.hpp"

class Simulator: public rclcpp::Node
{
    private:
        /* Hyperparameters */
        double dt, angle, range, vm;
        std::string world, tracker, target;

        /* Objects */
        std::unique_ptr<sim::Camera> sensor;
        std::unique_ptr<sim::RandomMap> map;
        std::unique_ptr<sim::Environment> env;
        std::unique_ptr<sim::Planner> planner;

        /* ROS */
        std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster;
        rclcpp::TimerBase::SharedPtr timer, mapping, detection, move;

        /* Publishers */
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr fov[2];
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planning;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectories[2];
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapper;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr detector;

        /* Subscribers */
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr controller;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rnd;

        /* Messages */
        bool moving = false;
        nav_msgs::msg::OccupancyGrid grid;
        sensor_msgs::msg::PointCloud2 scan;
        geometry_msgs::msg::TransformStamped transform;
    
    private:
        void step();

    public:
        Simulator(std::string);
};