/* @author: YueLin */

#include "planner/map.hpp"

namespace diff_track
{
    Map::Map(double w, double h, double r):
        resolution(r), width(w), height(h)
    {
        offset.width = std::round(w / (r * 2));
        offset.height = std::round(h / (r * 2));
        for(int m = 0; m < 3; m++)
            map[m] = cv::Mat::zeros(offset * 2, CV_8UC1);
        map[0] = ~*map;

        /* Initialize message */
        msg.info.resolution = r;
        msg.info.width = map->cols;
        msg.info.height = map->rows;
        msg.info.origin.position.x = -w / 2;
        msg.info.origin.position.y = -h / 2;
        msg.data.resize(map->cols * map->rows);
    }

    void Map::update(nav_msgs::msg::OccupancyGrid::SharedPtr grid)
    {
        /* Initialize map */
        for(int y = 0; y < map->rows; y++)
        {
            int z = y * map->cols;
            for(int x = 0; x < map->cols; x++)
                map->at<uchar>(y, x) = grid->data[x + z]? 0: 0xFF;
        }
        cv::rectangle(*map, cv::Rect(0, 0, map->cols, map->rows), 0, 1);

        /* Initialize ESDF */
        cv::distanceTransform(
            *map, *sdf, cv::DIST_L2, cv::DIST_MASK_PRECISE
        );
        cv::distanceTransform(
            ~*map, *(sdf + 1), cv::DIST_L2, cv::DIST_MASK_PRECISE
        );
        for(int y = 0; y < map->rows; y++)
            for(int x = 0; x < map->cols; x++)
                if(!map->at<uchar>(y, x))
                    sdf[0].at<float>(y, x) = -sdf[1].at<float>(y, x);
        *sdf *= resolution;
    }

    void Map::update(double range, double fov, double yaw,
                     const Eigen::Vector3d& self,
                     const Eigen::Vector3d& filter, 
                     const pcl::PointCloud<pcl::PointXYZ>& scan)
    {
        double r = 1. / resolution;
        cv::Point robot, point, cloud;
        
        /* Process point cloud */
        map[2] *= 0; 
        for(const pcl::PointXYZ& p: scan.points)
            if(std::fabs(p.x) < width && std::fabs(p.y) < height)
            {
                cloud.x = std::round(p.x * r) + offset.width;
                cloud.y = std::round(p.y * r) + offset.height;
                cv::circle(map[2], cloud, 2, 0xFF, -1);
            }

        /* Update map */
        robot.x = std::round(self.x() * r) + offset.width;
        robot.y = std::round(self.y() * r) + offset.height;
        for(double angle = deg2rad - fov; angle < fov; angle += deg2rad)
        {
            bool obstacle = false;
            double theta = angle + yaw;
            cloud.x = robot.x + std::round(r * range * std::cos(theta));
            cloud.y = robot.y + std::round(r * range * std::sin(theta));
            cv::LineIterator iterator = line(robot, cloud);
            for(int _ = iterator.count; _; --_)
            {
                point = iterator++.pos();
                map[1].at<uchar>(point) = 0xFF;
                if(map[2].at<uchar>(point))
                {
                    obstacle = true;
                    map[0].at<uchar>(point) = 0;
                }
                else if(obstacle) break;
                else map[0].at<uchar>(point) = 0xFF;
            }
        }

        /* Filter */
        if(filter.z() > 0)
        {
            point.x = std::round(filter.x() * r) + offset.width;
            point.y = std::round(filter.y() * r) + offset.height;
            cv::circle(*map, point, std::round(2 * filter.z() * r), 0xFF, -1);
        }
        
        /* Update ESDF */
        cv::distanceTransform(
            *map, *sdf, cv::DIST_L2, cv::DIST_MASK_PRECISE
        );
        cv::distanceTransform(
            ~*map, *(sdf + 1), cv::DIST_L2, cv::DIST_MASK_PRECISE
        );
        for(int y = 0; y < map->rows; y++)
            for(int x = 0; x < map->cols; x++)
                if(!map->at<uchar>(y, x))
                    sdf[0].at<float>(y, x) = -sdf[1].at<float>(y, x);
        *sdf *= resolution;
    }

    nav_msgs::msg::OccupancyGrid* Map::message(const std::string& frame, 
                                               double radius)
    {
        msg.header.frame_id = frame;
        for(int y = 0; y < map->rows; y++)
        {
            int z = y * map->cols;
            for(int x = 0; x < map->cols; x++)
                if(map[1].at<uchar>(y, x))
                {
                    float distance = sdf->at<float>(y, x);
                    msg.data[x + z] = radius >= distance? (
                        0 < distance? (radius - distance) / radius * 100: 100
                    ): 0;
                }
                else
                    msg.data[x + z] = -1;
        }
        return &msg;
    }
}
