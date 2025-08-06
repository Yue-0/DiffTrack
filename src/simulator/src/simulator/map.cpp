/* @author: YueLin */

#include <cmath>
#include <cstdlib>

#include "simulator/map.hpp"

namespace sim
{
    RandomMap::RandomMap(int seed, int obstalces, 
                         double w, double h, double r, double radius, 
                         double x1, double y1, double x2, double y2)
    {
        /* Initialize */
        width = w; height = h;
        resolution = r; r = 1 / r;
        cv::Size size(std::round(w * r), std::round(h * r));
        cv::Mat map = cv::Mat::ones(size, CV_8UC1) * 0xFF;
        offset = size / 2;

        /* Initialize message */
        msg.info.width = map.cols;
        msg.info.height = map.rows;
        msg.data.resize(size.area());
        msg.info.resolution = resolution;
        msg.info.origin.position.x = -width / 2;
        msg.info.origin.position.y = -height / 2;

        /* Generate random map */
        std::srand(seed);
        seed = std::sqrt(obstalces);
        int wh = std::sqrt(size.area()) / std::max(seed, 1) + 1;
        while(obstalces--)
            cv::rectangle(map, cv::Rect(
                std::rand() % size.width,
                std::rand() % size.height,
                std::max(std::rand() % wh, wh / seed),
                std::max(std::rand() % wh, wh / seed)
            ), 0, -1);
        cv::rectangle(map, cv::Rect(cv::Point(
            std::round((std::min(x1, x2) - 0xa * radius) * r) + offset.width,
            std::round((std::min(y1, y2) - 0xa * radius) * r) + offset.height
        ), cv::Point(
            std::round((std::max(x1, x2) + 0xa * radius) * r) + offset.width,
            std::round((std::max(y1, y2) + 0xa * radius) * r) + offset.height
        )), 0xFF, -1);

        /* Create ESDF */
        cv::Mat esdf;
        cv::distanceTransform(
            map, data, cv::DIST_L2, cv::DIST_MASK_PRECISE
        );
        cv::distanceTransform(
            ~map, esdf, cv::DIST_L2, cv::DIST_MASK_PRECISE
        );
        for(int y = 0; y < size.height; y++)
            for(int x = 0; x < size.width; x++)
                if(!map.at<uchar>(y, x))
                    data.at<float>(y, x) = -esdf.at<float>(y, x);
        data *= resolution;
    }

    geometry_msgs::msg::Pose RandomMap::goal(int seed) const
    {
        /* Generate random point */
        cv::Point point;
        std::srand(seed);
        do
        {
            point.x = std::rand() % data.cols;
            point.y = std::rand() % data.rows;
        }
        while(data.at<float>(point) <= 0);

        /* Convert to PoseStamped */
        geometry_msgs::msg::Pose pose;
        pose.position.x = (point.x - offset.width) * resolution;
        pose.position.y = (point.y - offset.height) * resolution;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 0;
        return pose;
    }

    nav_msgs::msg::OccupancyGrid* RandomMap::message(const std::string& frame)
    {
        msg.header.frame_id = frame;
        for(int y = 0; y < data.rows; y++)
        {
            int z = y * data.cols;
            for(int x = 0; x < data.cols; x++)
                msg.data[x + z] = data.at<float>(y, x) <= 0? 100: 0;
        }
        return &msg;
    }
}
