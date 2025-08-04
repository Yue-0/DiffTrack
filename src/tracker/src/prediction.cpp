/* @author: YueLin */

#include <chrono>
#include <functional>

#include "prediction.hpp"

Predictor::Predictor(std::string name): Node(name)
{
    /* Declare hyperparameters */
    declare_parameter<int>("m");
    declare_parameter<double>("tau");
    dt = 1. / declare_parameter<double>("hz");

    /* Initialize Bezier curve */
    bezier = std::make_unique<diff_track::Bezier>(
        1 + 2 * get_parameter("m").as_int(), get_parameter("tau").as_double()
    );

    /*  Initialize trajectory predictor */
    predictor = std::make_unique<diff_track::Predictor>(bezier.get(), 1. / dt);
    predictor->reset();

    /* Create publishers */
    publishers[0] = create_publisher<nav_msgs::msg::Path>("/target/bezier", 1);
    publishers[1] = create_publisher<nav_msgs::msg::Path>("/target/predict", 1);

    /* Create subscriber */
    sub = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target/detection", 1,
        std::bind(&Predictor::predict, this, std::placeholders::_1)
    );
}

void Predictor::predict(const geometry_msgs::msg::PoseStamped::SharedPtr det)
{
    if(det->pose.orientation.w < 0)
    {
        predictor->reset(); 
        // TODO: target lost >_<
    }
    else
    {
        /* Predict trajectory */
        predictor->enqueue(
            det->pose.position.x, 
            det->pose.position.y
        );
        predictor->predict();

        /* Publish Bezier curve */
        message.header.frame_id = det->header.frame_id;
        message.poses.resize(bezier->n + 1);
        for(int p = 0; p <= bezier->n; p++)
        {
            message.poses[p].pose.position.x = bezier->control(0, p);
            message.poses[p].pose.position.y = bezier->control(1, p);
        }
        publishers[0]->publish(message);

        /* Publish predicted trajectory */
        message.poses.clear();
        for(double t = 0; t <= bezier->duration; t += dt)
        {
            geometry_msgs::msg::PoseStamped pose;
            Eigen::Vector2d p = bezier->get(t);
            pose.pose.position.x = p.x();
            pose.pose.position.y = p.y();
            message.poses.push_back(pose);
        }
        publishers[1]->publish(message);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Predictor>("predictor"));
    rclcpp::shutdown();
    return 0;
}
