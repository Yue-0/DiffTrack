/* @author: YueLin */

#include <chrono>
#include <functional>

#include <pcl_conversions/pcl_conversions.h>

#include "geometry_msgs/msg/quaternion.hpp"

#include "planning.hpp"

inline double quaternion2yaw(geometry_msgs::msg::Quaternion q)
{
    double x = q.x * q.x, y = q.y * q.y, z = q.z * q.z, w = q.w * q.w;
    double s = 2 * (q.w * q.y - q.x * q.z) / (x + y + z + w);
    if(std::fabs(s) >= 1 - 1e-6) 
        return std::copysign(2 * std::atan2(q.y, q.x), s);
    return std::atan2(2 * (q.x * q.y + q.z * q.w), w + x - y - z);
}

Tracker::Tracker(std::string name): Node(name)
{
    /* Declare hyperparameters */
    range = declare_parameter<double>("tracker.range");
    double dt = declare_parameter<double>("planner.delta_t");
    tracker.z() = declare_parameter<double>("tracker.radius");
    fov = declare_parameter<double>("tracker.fov") * deg2rad / 2;
    message.header.frame_id = declare_parameter<std::string>("map.name");

    declare_parameter<int>("bezier.m");
    declare_parameter<double>("bezier.tau");

    declare_parameter<double>("map.size");
    declare_parameter<double>("map.resolution");

    declare_parameter<double>("tracker.max_vel");
    declare_parameter<double>("tracker.max_acc");
    declare_parameter<double>("tracker.max_alpha");
    declare_parameter<double>("tracker.max_omega");

    declare_parameter<int>("planner.kappa");
    declare_parameter<int>("planner.samples");
    declare_parameter<int>("planner.num_angles");
    declare_parameter<double>("planner.epsilon");
    declare_parameter<double>("planner.observation_distance");

    declare_parameter<double>("planner.lambda_a");
    declare_parameter<double>("planner.lambda_d");
    declare_parameter<double>("planner.lambda_o");
    declare_parameter<double>("planner.lambda_c");
    declare_parameter<double>("planner.lambda_f");
    declare_parameter<double>("planner.lambda_s");

    declare_parameter<int>("lbfgs.past");
    declare_parameter<int>("lbfgs.memory");
    declare_parameter<double>("lbfgs.delta");
    declare_parameter<double>("lbfgs.wolfe");
    declare_parameter<double>("lbfgs.armijo");
    declare_parameter<double>("lbfgs.epsilon");
    declare_parameter<int>("lbfgs.iterations");
    declare_parameter<double>("lbfgs.max_step");

    /* Initialize states */
    tigger = 0;
    pointer = 0;
    target.setZero();
    trajectory.resize(5, 0);
    tracker.head(2).setZero();
    state = Eigen::VectorXd::Zero(5);

    /* Initialize map */
    map = std::make_unique<diff_track::Map>(
        get_parameter("map.size").as_double(),
        get_parameter("map.size").as_double(),
        get_parameter("map.resolution").as_double()
    );

    /* Initialize Bezier curve */
    bezier = std::make_unique<diff_track::Bezier>(
        get_parameter("bezier.m").as_int() * 2 + 1, 
        get_parameter("bezier.tau").as_double()
    );

    /* Initialize hybrid A* planner */
    planner = std::make_unique<diff_track::HybridAStar>(
        map.get(), bezier.get(),

        /* Hyperparameters */
        get_parameter("planner.num_angles").as_int(),
        get_parameter("planner.samples").as_int(),
        
        /* Robot settings */
        range, tracker.z(), dt,

        /* Motion constraints */
        get_parameter("tracker.max_vel").as_double(),
        get_parameter("tracker.max_acc").as_double(),

        /* Range of perception */
        get_parameter("planner.observation_distance").as_double(),
        get_parameter("planner.epsilon").as_double(),
        bezier->duration
    );

    /* Initialize model predictive controller */
    mpc = std::make_unique<diff_track::ModelPredictiveControl>(
        map.get(), bezier.get(),

        /* Robot settings */
        2 * tracker.z(), dt,
        get_parameter("planner.observation_distance").as_double(),

        /* Weights of cost terms */
        get_parameter("planner.lambda_a").as_double(),
        get_parameter("planner.lambda_d").as_double(),
        get_parameter("planner.lambda_o").as_double(),
        get_parameter("planner.lambda_c").as_double(),
        get_parameter("planner.lambda_f").as_double(),
        get_parameter("planner.lambda_s").as_double(),

        /* Motion constraints */
        get_parameter("tracker.max_vel").as_double(),
        get_parameter("tracker.max_acc").as_double(),
        get_parameter("tracker.max_omega").as_double(),
        get_parameter("tracker.max_alpha").as_double(),
        
        /* Hyperparameters for L-BFGS */
        get_parameter("lbfgs.past").as_int(),
        get_parameter("lbfgs.memory").as_int(),
        get_parameter("planner.kappa").as_int(),
        get_parameter("lbfgs.epsilon").as_double(),
        get_parameter("lbfgs.max_step").as_double(),
        get_parameter("lbfgs.delta").as_double(),
        get_parameter("lbfgs.epsilon").as_double(),
        get_parameter("lbfgs.iterations").as_int(),
        get_parameter("lbfgs.wolfe").as_double(),
        get_parameter("lbfgs.armijo").as_double()
    );

    /* Create publishers */
    publisher = create_publisher<nav_msgs::msg::Path>("/tracker/plan", 1);
    cmd = create_publisher<geometry_msgs::msg::Twist>("/tracker/cmd_vel", 1);
    mapper = create_publisher<nav_msgs::msg::OccupancyGrid>("/tracker/map", 1);

    /* Create localization subscriber */
    localization = create_subscription<nav_msgs::msg::Odometry>(
        "/tracker/odom", 1, 
        [this](const nav_msgs::msg::Odometry::SharedPtr odom){
            state.x() = odom->pose.pose.position.x;
            state.y() = odom->pose.pose.position.y;
            state.z() = quaternion2yaw(odom->pose.pose.orientation);
            if(!trajectory.cols()) trajectory = state;
            tracker.head(2) = state.head(2);
            tigger |= 1;
        }
    );

    /* Create sensor subscriber */
    global = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 1, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr grid){
            if(tigger & 8) return;
            map->update(grid);
            tigger |= 8;
        }
    );
    sensor = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/tracker/scan", 1, 
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr pc){
            if(!(tigger & 1)) return;
            pcl::fromROSMsg(*pc, cloud);
            map->update(range, fov, state.z(), tracker, target, cloud);
            mapper->publish(*map->message(
                message.header.frame_id, tracker.z()
            ));
            tigger |= 2;
        }
    );

    /* Create target detection subscriber */
    detect = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target/detection", 1, 
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr detection){
            if(detection->pose.orientation.w > 0)
            {
                tigger |= 4;
                target.x() = detection->pose.position.x;
                target.y() = detection->pose.position.y;
                target.z() = detection->pose.position.z * 1.5;
            }
            else
                target.z() = -1;
        }
    );

    /* Create subscriber for trajectory planning */
    planning = create_subscription<nav_msgs::msg::Path>(
        "/target/bezier", 1, 
        std::bind(&Tracker::callback, this, std::placeholders::_1)
    );

    /* Create timer for replanning */
    int ms = 1e3 * bezier->time;
    replan = create_wall_timer(std::chrono::milliseconds(ms), [this](){
        if((tigger & 7) == 7 && 
           pointer + 1 >= trajectory.cols() &&
           bezier->duration < (rclcpp::Clock().now() - time).seconds())
        {
            pointer = 0;
            twist.linear.x = 0;
            twist.angular.z = 0;
            cmd->publish(twist);
            mpc->visibility(false);
            state.tail(2).setZero();
            trajectory = planner->reacquire(false, state);
        }
    });

    /* Create controller */
    ms = 1e3 * dt;
    controller = create_wall_timer(
        std::chrono::milliseconds(ms), 
        std::bind(&Tracker::ctrl, this)
    );
}

void Tracker::ctrl()
{
    if((tigger & 7) != 7)
        return;

    /* Set time stamp */
    rclcpp::Time now = rclcpp::Clock().now();
    mpc->clock((now - time).seconds());
    message.header.stamp = now;
    
    /* Model predictive control */
    Eigen::MatrixXd control;
    int length = trajectory.cols();
    if(length > pointer + 1)
        control = mpc->control(state, trajectory.rightCols(length - pointer++));
    else 
        control = mpc->control(state, state);

    /* Publish velocity */
    float d0 = map->at(control.col(0).head(2));
    float d1 = map->at(control.col(1).head(2));
    if(d1 <= 0 && d1 < d0)
    {
        control = mpc->control(state, state);
        d1 = map->at(control.col(1).head(2));
        if(d1 <= 0 && d1 < d0)
            control.col(1).tail(2).setZero();
    }
    state[3] = twist.linear.x = control(3, 1);
    state[4] = twist.angular.z = control(4, 1);
    cmd->publish(twist);

    /* Publish trajectory */
    bool colussion = false;
    length = control.cols();
    for(int p = 1; p < length; p++)
        if((colussion = map->at(control.col(p).head(2)) < tracker.z()))
        {
            length = p; break;
        }
    message.poses.resize(length);
    for(int p = 0; p < length; p++)
    {
        message.poses[p].pose.position.x = control(0, p);
        message.poses[p].pose.position.y = control(1, p);
        message.poses[p].pose.orientation.z = std::sin(control(2, p) / 2);
        message.poses[p].pose.orientation.w = std::cos(control(2, p) / 2);
    }
    publisher->publish(message);

    /* Replanning */
    if(colussion && !mpc->visibility())
    {
        pointer = 0;
        twist.linear.x = 0;
        twist.angular.z = 0;
        cmd->publish(twist);
        state.tail(2).setZero();
        trajectory = planner->reacquire(true, state);
    }
}

void Tracker::callback(const nav_msgs::msg::Path::SharedPtr b)
{
    if((tigger & 7) != 7)
        return;
    
    /* Update Bezier curve */
    int num = b->poses.size();
    time = rclcpp::Clock().now();
    for(int p = 0; p < num; p++)
    {
        bezier->control(0, p) = b->poses[p].pose.position.x;
        bezier->control(1, p) = b->poses[p].pose.position.y;
    }

    /* Reset trajectory */
    if(pointer + 1 >= trajectory.cols())
    {
        pointer = 0;
        trajectory = state;
    }

    /* Path planning */
    if(planner->plan(trajectory, state))
        pointer = 0;

    /* Set visibility of MPC */
    mpc->visibility(true);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tracker>("tracker"));
    rclcpp::shutdown();
    return 0;
}
