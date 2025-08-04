/* @author: YueLin */

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "predictor/predictor.hpp"

class Predictor: public rclcpp::Node
{
    private:
        /* Hyperparameter */
        double dt;

        /* Objects */
        std::unique_ptr<diff_track::Bezier> bezier;
        std::unique_ptr<diff_track::Predictor> predictor;

        /* Publishers */
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publishers[2];

        /* Subscriber */
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub;

        /* Message */
        nav_msgs::msg::Path message;
    
    private:
        void predict(const geometry_msgs::msg::PoseStamped::SharedPtr);

    public:
        Predictor(std::string);
};