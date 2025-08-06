/* @author: YueLin */

#include <chrono>
#include <functional>

#include <pcl_conversions/pcl_conversions.h>

#include "simulation.hpp"

Simulator::Simulator(std::string name): Node(name)
{
    /* Declare hyperparameters */
    dt = 1. / declare_parameter<double>("hz");
    range = declare_parameter<double>("tracker.range");
    world = declare_parameter<std::string>("map.name");
    target = declare_parameter<std::string>("target.name");
    tracker = declare_parameter<std::string>("tracker.name");
    angle = declare_parameter<double>("tracker.fov") * PI / 360;
    vm = declare_parameter<double>("target.max_vel") / std::sqrt(2);

    declare_parameter<int>("map.seed");
    declare_parameter<int>("map.obstacles");
    declare_parameter<double>("map.width");
    declare_parameter<double>("map.height");
    declare_parameter<double>("map.resolution");
    
    declare_parameter<double>("target.x");
    declare_parameter<double>("target.y");
    declare_parameter<double>("target.yaw");
    declare_parameter<double>("target.radius");
    declare_parameter<double>("target.max_acc");

    declare_parameter<double>("tracker.x");
    declare_parameter<double>("tracker.y");
    declare_parameter<double>("tracker.yaw");
    declare_parameter<double>("tracker.radius");

    /* Initialize random map */
    map = std::make_unique<sim::RandomMap>(
        get_parameter("map.seed").as_int(),
        get_parameter("map.obstacles").as_int(),
        get_parameter("map.width").as_double(),
        get_parameter("map.height").as_double(),
        get_parameter("map.resolution").as_double(),
        get_parameter("tracker.radius").as_double(),
        get_parameter("tracker.x").as_double(),
        get_parameter("tracker.y").as_double(),
        get_parameter("target.x").as_double(),
        get_parameter("target.y").as_double()
    );

    /* Initialize simulation environment */
    env = std::make_unique<sim::Environment>(
        world, map.get(),
        get_parameter("target.x").as_double(),
        get_parameter("target.y").as_double(),
        get_parameter("target.yaw").as_double(),
        get_parameter("target.radius").as_double(),
        get_parameter("tracker.x").as_double(),
        get_parameter("tracker.y").as_double(),
        get_parameter("tracker.yaw").as_double(),
        get_parameter("tracker.radius").as_double()
    );

    /* Initialize limited FoV sensor */
    sensor = std::make_unique<sim::Camera>(tracker, env->tracker, angle, range);

    /* Initialize path planner for the target */
    planner = std::make_unique<sim::Planner>(
        &env->target, map.get(), dt * 2, vm,
        get_parameter("target.max_acc").as_double()
    );

    /* Initialize messages */
    scan.header.frame_id = world;
    transform.header.frame_id = world;

    /* Create publishers and broadcaster */
    planning = create_publisher<nav_msgs::msg::Path>(
        "/target/plan", 1
    );
    trajectories[0] = create_publisher<nav_msgs::msg::Path>(
        "/target/trajectory", 1
    );
    trajectories[1] = create_publisher<nav_msgs::msg::Path>(
        "/tracker/trajectory", 1
    );
    odometry = create_publisher<nav_msgs::msg::Odometry>(
        "/tracker/odom", 1
    );
    detector = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/target/detection", 1
    );
    *fov = create_publisher<nav_msgs::msg::Path>("/tracker/fov", 1);
    mapper = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
    broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    fov[1] = create_publisher<nav_msgs::msg::Path>("/tracker/fov/lost", 1);
    depth = create_publisher<sensor_msgs::msg::PointCloud2>("/tracker/scan", 1);

    /* Create subscribers */
    controller = create_subscription<geometry_msgs::msg::Twist>(
        "/tracker/cmd_vel", 1, 
        [this](const geometry_msgs::msg::Twist::SharedPtr cmd){
            env->tracker.control(*cmd);
        }
    );
    goal = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 1, 
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr goal){
            planner->plan(goal->pose);
        }
    );
    rnd = create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 1, 
        [this](const geometry_msgs::msg::PointStamped::SharedPtr){
            moving = !moving;
        }
    );

    /* Initialize timer */
    int ms = dt * 1e3;
    timer = create_wall_timer(
        std::chrono::milliseconds(ms),
        std::bind(&Simulator::step, this)
    );

    /* Detect target */
    ms = 1e3 / declare_parameter<double>("tracker.fps");
    detection = create_wall_timer(std::chrono::milliseconds(ms), [this](){
        detector->publish(*env->detect(angle, range));
    });

    /* Random movement of the target */
    move = create_wall_timer(std::chrono::milliseconds(ms), [this](){
        if(moving && 1e-2 > std::fabs(env->target.vel.yaw) + hypot2(
            env->target.vel.x, env->target.vel.y
        ))
            planner->plan(map->goal(rclcpp::Clock().now().seconds()));
    });

    /* Publish map */
    mapping = create_wall_timer(std::chrono::seconds(1), [this](){
        if(grid.header.frame_id != world)
            grid = *map->message(world);
        mapper->publish(grid);
    });
}

void Simulator::step()
{
    /* Control the target */
    planning->publish(*planner->message(world));
    env->target.control(planner->control());

    /* Robot movement */
    env->step(dt);

    /* Publish the tracker's odometry */
    odometry->publish(env->tracker.odom(tracker, world));

    /* Boardcast transforms */
    transform.header.stamp = now();
    env->target.transform(target, broadcaster.get(), transform);
    env->tracker.transform(tracker, broadcaster.get(), transform);

    /* Publish trajectories */
    trajectories[0]->publish(*env->target.trajectory(world));
    trajectories[1]->publish(*env->tracker.trajectory(world));

    /* Perception */
    sensor->update(map.get(), env->tracker, env->target);
    pcl::toROSMsg(*sensor->depth(), scan);
    scan.header.frame_id = world;
    depth->publish(scan);

    /* Visualize FoV */
    nav_msgs::msg::Path* view = sensor->view();
    if(env->lost())
    {
        fov[1]->publish(*view);
        fov[0]->publish(*(view + 1));
    }
    else
    {
        fov[0]->publish(*view);
        fov[1]->publish(*(view + 1));
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Simulator>("simulator"));
    rclcpp::shutdown();
    return 0;
}
