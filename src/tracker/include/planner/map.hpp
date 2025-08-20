/* @author: YueLin */

#pragma once

#include <cmath>

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>

#include "nav_msgs/msg/occupancy_grid.hpp"

const double PI = std::acos(-1);
const double deg2rad = PI / 180;

/* Clip rad to (-PI, PI] */
inline double normalize(double rad)
{
    if(rad > PI || rad <= -PI)
        return 2 * std::floor(0.5 - rad / (2 * PI)) * PI + rad;
    return rad;
}

namespace diff_track
{
    class Map
    {
        private:
            cv::Mat map[3], sdf[2];
            nav_msgs::msg::OccupancyGrid msg;
        
        public:
            cv::Size offset;
            double resolution;
            double width, height;
        
        public:
            Map(double, double, double);

            void update(nav_msgs::msg::OccupancyGrid::SharedPtr);

            void update(double, double, double,
                        const Eigen::Vector3d&, 
                        const Eigen::Vector3d&,
                        const pcl::PointCloud<pcl::PointXYZ>&);

            nav_msgs::msg::OccupancyGrid* message(const std::string&, double);

            cv::LineIterator line(const cv::Point& p1, const cv::Point& p2) 
            const
            {
                return cv::LineIterator(*map, p1, p2);
            }

            cv::Point decode(int code) const
            {
                cv::Point point;
                point.y = code / map->cols;
                point.x = code - map->cols * point.y;
                return point;
            }

            int encode(const cv::Point& point) const
            {
                return map->cols * point.y + point.x;
            }
            
            cv::Point encode(const Eigen::Vector2d& point) const
            {
                return cv::Point(
                    std::max(std::min(
                        map->cols - 1.,
                        std::round(point.x() / resolution) + offset.width
                    ), 0.), 
                    std::max(std::min(
                        map->rows - 1.,
                        std::round(point.y() / resolution) + offset.height
                    ), 0.)
                );
            }

            Eigen::Vector2d decode(const cv::Point& point) const
            {
                return Eigen::Vector2d(
                    resolution * (point.x - offset.width),
                    resolution * (point.y - offset.height)
                );
            }

            float at(const cv::Point& point) const
            {
                return sdf->at<float>(point);
            }

            float at(const Eigen::Vector2d& point) const
            {
                return sdf->at<float>(encode(point));
            }

            bool known(const Eigen::Vector2d& point) const
            {
                return (map + 1)->at<uchar>(encode(point));
            }

            float esdf(const Eigen::Vector2d& point, Eigen::Vector2d& gradient)
            const 
            {
                double x = point.x() / resolution + offset.width;
                double y = point.y() / resolution + offset.height;
                
                /* Linear interpolation */
                int x1 = x, y1 = y;
                int x2 = x1 + 1, y2 = y1 + 1;
                double u = x - x1, v = y - y1, u_ = x2 - x, v_ = y2 - y;
                
                /* Boundary check */
                if(std::min(x1, y1) < 0 || x2 >= sdf->cols || y2 >= sdf->rows)
                {
                    gradient.setZero(); return 0;
                }
                
                /* Linear interpolation for gradient */
                gradient.x() = v * sdf->at<float>(y2, x2)
                             - v * sdf->at<float>(y2, x1)
                             + v_ * sdf->at<float>(y1, x2)
                             - v_ * sdf->at<float>(y1, x1);
                gradient.y() = u * sdf->at<float>(y2, x2)
                             - u * sdf->at<float>(y1, x2)
                             + u_ * sdf->at<float>(y2, x1)
                             - u_ * sdf->at<float>(y1, x1);

                /* Linear interpolation for ESDF value */
                return u * v * sdf->at<float>(y2, x2)
                     + u * v_ * sdf->at<float>(y1, x2)
                     + u_ * v * sdf->at<float>(y2, x1)
                     + u_ * v_ * sdf->at<float>(y1, x1);
            }
    };
}
