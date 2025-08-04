/* @author: YueLin */

#include "simulator/env.hpp"

namespace sim
{
    Environment::Environment(std::string name,
                             RandomMap* rm,
                             double target_x,
                             double target_y,
                             double target_z,
                             double target_r,
                             double tracker_x,
                             double tracker_y,
                             double tracker_z,
                             double tracker_r): 
        target(target_x, target_y, target_z, target_r),
        tracker(tracker_x, tracker_y, tracker_z, tracker_r), map(rm)
    {
        detection.header.frame_id = name;
    }
    
    geometry_msgs::msg::PoseStamped* Environment::detect(double fov, 
                                                         double range)
    {
        /* Check observation distance and angle */
        bool capture = hypot2(
            target.pose.x - tracker.pose.x, 
            target.pose.y - tracker.pose.y
        ) <= range * range && std::fabs(normalize(std::atan2(
            target.pose.y - tracker.pose.y,
            target.pose.x - tracker.pose.x
        ) - tracker.pose.yaw)) <= fov;

        /* Check occlusion */
        if(capture)
        {
            cv::LineIterator line = map->line(
                map->encode(target.pose.x, target.pose.y),
                map->encode(tracker.pose.x, tracker.pose.y)
            );
            for(int _ = line.count; capture && _; --_)
                capture &= map->index(line++.pos()) > 0;
        }

        /* Construct message */
        if(capture)
        {
            detection.pose.orientation.w = 1;
            detection.pose.position.x = target.pose.x;
            detection.pose.position.y = target.pose.y;
            detection.pose.position.z = target.radius;
        }
        else
            detection.pose.orientation.w = -1;
        return &detection;
    }
}
