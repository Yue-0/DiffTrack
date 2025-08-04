/* @author: YueLin */

#include "simulator/robot.hpp"

namespace sim
{
    Robot::Robot(double x, double y, double yaw, double r):
        pose(x, y, normalize(yaw)), vel(0, 0, 0), radius(r) {}

    void Robot::move(double dt)
    {
        if(std::fabs(vel.yaw) < 1e-2)
        {
            double sin = std::sin(pose.yaw);
            double cos = std::cos(pose.yaw);
            pose.x += (vel.x * cos - vel.y * sin) * dt;
            pose.y += (vel.x * sin + vel.y * cos) * dt;
        }
        else
        {
            double yaw = normalize(pose.yaw + vel.yaw * dt);
            double sin = std::sin(yaw) - std::sin(pose.yaw);
            double cos = std::cos(pose.yaw) - std::cos(yaw);
            pose.x += (vel.x * sin - vel.y * cos) / vel.yaw;
            pose.y += (vel.x * cos + vel.y * sin) / vel.yaw;
            pose.yaw = yaw;
        }
    }

    void Robot::transform(const std::string& frame,
                          tf2_ros::TransformBroadcaster* tb,
                          geometry_msgs::msg::TransformStamped& tf)
    {
        tf.child_frame_id = frame;
        tf.transform.translation.x = pose.x;
        tf.transform.translation.y = pose.y;
        tf.transform.rotation.z = std::sin(pose.yaw / 2);
        tf.transform.rotation.w = std::cos(pose.yaw / 2);
        tb->sendTransform(tf);
    }

    nav_msgs::msg::Odometry Robot::odom(const std::string& name,
                                        const std::string& frame)
    {
        /* Initialize message */
        nav_msgs::msg::Odometry odometry;
        odometry.header.frame_id = frame;
        odometry.child_frame_id = name;

        /* Set pose */
        odometry.pose.pose.position.x = pose.x;
        odometry.pose.pose.position.y = pose.y;
        odometry.pose.pose.orientation.z = std::sin(pose.yaw / 2);
        odometry.pose.pose.orientation.w = std::cos(pose.yaw / 2);
        
        /* Set velocity */
        double sin = odometry.pose.pose.orientation.z
                   * odometry.pose.pose.orientation.w * 2;
        double cos = odometry.pose.pose.orientation.w
                   * odometry.pose.pose.orientation.w * 2 - 1;
        odometry.twist.twist.linear.x = vel.x * cos - vel.y * sin;
        odometry.twist.twist.linear.y = vel.x * sin + vel.y * cos;
        odometry.twist.twist.angular.z = vel.yaw;

        return odometry;
    }

    nav_msgs::msg::Path* Robot::trajectory(const std::string& frame)
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.pose.position.x = pose.x;
        ps.pose.position.y = pose.y;
        ps.header.frame_id = frame;
        
        if(path.poses.empty())
        {
            path.poses.push_back(ps);
            path.header.frame_id = frame;
        }
        else
        {
            geometry_msgs::msg::Point 
            p = path.poses.back().pose.position;
            if(hypot2(p.x - pose.x, p.y - pose.y) > 1e-2)
                path.poses.push_back(ps);
        }
        return &path;
    }
}
