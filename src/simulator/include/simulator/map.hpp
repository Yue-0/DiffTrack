/* @author: YueLin */

#pragma once

#include "opencv2/opencv.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace sim
{
    class RandomMap
    {
        private:
            cv::Mat data;
            cv::Size offset;
            nav_msgs::msg::OccupancyGrid msg;
        
        public:
            double width, height, resolution;
            
        public:
            RandomMap(int, int, double, double, double,
                      double, double, double, double, double);

            cv::Point encode(double x, double y) const
            {
                return cv::Point(
                    std::max(std::min(
                        data.cols - 1., 
                        std::round(x / resolution) + offset.width
                    ), 0.), 
                    std::max(std::min(
                        data.rows - 1., 
                        std::round(y / resolution) + offset.height
                    ), 0.)
                );
            }

            int rows() const {return data.rows;}
            int cols() const {return data.cols;}
            float index(int x, int y) const {return data.at<float>(y, x);}
            float index(cv::Point point) const {return data.at<float>(point);}
            float cost(double x, double y) const {return index(encode(x, y));}

            cv::LineIterator line(const cv::Point& p1, const cv::Point& p2) 
            const
            {
                return cv::LineIterator(data, p1, p2);
            }

            geometry_msgs::msg::Pose goal(int) const;
            
            nav_msgs::msg::OccupancyGrid* message(const std::string&);
    };
}
