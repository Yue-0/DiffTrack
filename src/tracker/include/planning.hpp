/* @author: YueLin */

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "planner/mpc.hpp"
#include "planner/astar.hpp"

class Tracker: public rclcpp::Node
{
    private:
        /* Hyperparameters */
        double fov, range;

        /* States */
        int pointer;
        short tigger;  // Bit: global map, detection, mapping, localization
        Eigen::VectorXd state;            // x, y, yaw, vel, omega
        Eigen::MatrixXd trajectory;       // (5, n)
        Eigen::Vector3d target, tracker;  // x, y, radius

        /* Objects */
        rclcpp::Time time;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        std::unique_ptr<diff_track::Map> map;
        std::unique_ptr<diff_track::Bezier> bezier;
        std::unique_ptr<diff_track::HybridAStar> planner;
        std::unique_ptr<diff_track::ModelPredictiveControl> mpc;

        /* Timer */
        rclcpp::TimerBase::SharedPtr replan, controller;

        /* Publishers */
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapper;

        /* Subscriber */
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr planning;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr detect;

        /* Message */
        nav_msgs::msg::Path message;
        geometry_msgs::msg::Twist twist;
    
    private:
        void ctrl();
        void callback(const nav_msgs::msg::Path::SharedPtr);

    public:
        Tracker(std::string);
};