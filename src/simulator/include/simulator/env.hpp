/* @author: YueLin */

#include "simulator/map.hpp"
#include "simulator/robot.hpp"

namespace sim
{
    class Environment
    {
        public:
            Robot target;
            Robot tracker;
            RandomMap* map;
        
        private:
            geometry_msgs::msg::PoseStamped detection;
        
        public:
            Environment(std::string, RandomMap*,
                        double, double, double, double,
                        double, double, double, double);
        
        private:
            void clip(Robot& robot)
            {
                robot.pose.x = std::max(
                    std::min(robot.pose.x, map->width / 2), -map->width / 2.
                );
                robot.pose.y = std::max(
                    std::min(robot.pose.y, map->height / 2), -map->height / 2
                );
            }

            void step(Robot& robot, double dt) {robot.move(dt); clip(robot);}
            
        public:
            geometry_msgs::msg::PoseStamped* detect(double, double);

            bool lost() const {return detection.pose.orientation.w < 0;}

            void step(double dt)
            {
                SE2 pose = tracker.pose;
                step(target, dt); step(tracker, dt);
                if(map->cost(tracker.pose.x, tracker.pose.y) < tracker.radius)
                    tracker.pose = pose;
            }
    };
}
