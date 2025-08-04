/* @author: YueLin */

#include <queue>
#include <utility>

#include "planner/astar.hpp"

namespace diff_track
{
    HybridAStar::~HybridAStar()
    {
        delete[] g;
        delete[] time;
        delete[] parent;
        delete[] closed;
    }

    HybridAStar::HybridAStar(Map* world, 
                             Bezier* prediction,
                             int angle, int num, 
                             double r, double t, 
                             double max_vel, double max_omega,
                             double distance, double e, double tau)
    :   map(world),
        bezier(prediction),
        angles(angle), radius(r),
        dt(t), vm(max_vel), wm(max_omega),
        dv(max_vel / num), dw(max_omega / num),
        ob(distance), epsilon(e), duration(tau)
    {
        size = angle
             * std::round(world->width / world->resolution)
             * std::round(world->height / world->resolution);
        states.resize(5, size);
        closed = new bool[size];
        parent = new int[size];
        time = new double[size];
        g = new double[size];
    }

    bool HybridAStar::plan(Eigen::MatrixXd& trajectory,
                           const Eigen::VectorXd& tracker)
    {
        /* Get predicted trajectory of the target */
        Eigen::Matrix2Xd prediction = bezier->trajectory();

        /* Check replan */
        int n = prediction.cols();
        if(visible(prediction.col(n - 1), trajectory.topRightCorner(3, 1)))
            return false;
        
        /* Try to plan */
        while(n--)
        {
            /* Generate a proposal observation point */
            Eigen::Vector3d goal = proposal(prediction.col(n), tracker.head(2));
            
            /* Path search */
            if(!std::isinf(goal.z()))
            {
                Eigen::MatrixXd path = search(tracker, prediction.col(n), goal);
                if(path.cols())
                {
                    path.col(0) = tracker;
                    trajectory = path;
                    return true;
                }
            }
        }

        /* Planning failed */
        std::cout << "Planning failed" << std::endl;
        return false;
    }

    void HybridAStar::trigonometric(double angle)
    {
        trigon.x() = std::cos(angle);
        trigon.y() = std::sin(angle);
    }

    int HybridAStar::hash(const Eigen::Vector3d& point)
    {
        cv::Point code = map->encode(point.head(2));
        return 4 * map->offset.area() * std::floor(
            0.5 * angles * (PI - point.z()) / PI
        ) + 2 * code.y * map->offset.width + code.x;
    }

    double HybridAStar::f(double g, 
                          const Eigen::Vector3d& tracker, 
                          const Eigen::Vector3d& observation)
    {
        Eigen::Vector3d d = observation - tracker;
        return g + radius * std::fabs(normalize(d.z())) + d.head(2).norm();
    }

    Eigen::Vector2d HybridAStar::bfs(const Eigen::Vector2d& start)
    {
        /* Check obstacle */
        if(map->at(start) > radius)
            return start;
        cv::Size sz = map->offset * 2;
        cv::Point point = map->encode(start);
        
        /* Initialize queue and arrays */
        std::queue<int> queue;
        int index = sz.area();
        std::fill_n(closed, index, false);

        /* Push the first point into the queue */
        index = sz.width * point.y + point.x;
        closed[index] = true;
        queue.push(index);

        /* BFS */
        while(!queue.empty())
        {
            /* Dequeue a point */
            cv::Point p;
            index = queue.front(); queue.pop();
            p.y = index / sz.width; p.x = index - p.y * sz.width;

            /* Expand */
            for(int dx = -1; dx <= 1; dx++)
            {
                int x = p.x + dx;
                if(x < 0 || x >= sz.width)
                    continue;
                for(int dy = -1; dy <= 1; dy++)
                {
                    int y = p.y + dy;
                    if(y < 0 || y >= sz.height)
                        continue;
                    int idx = sz.width * y + x;
                    if(!closed[idx])
                    {
                        if(map->at(cv::Point(x, y)) > radius)
                            return Eigen::Vector2d(
                                map->resolution * (x - map->offset.width),
                                map->resolution * (y - map->offset.height)
                            );
                        closed[idx] = true;
                        queue.push(idx);
                    }
                }
            }
        }
        return start;
    }

    Eigen::MatrixXd HybridAStar::search(Eigen::VectorXd tracker,
                                        const Eigen::Vector2d& target,
                                        const Eigen::Vector3d& observation)
    {
        /* Find a safe start point */
        tracker.head(2) = bfs(tracker.head(2));

        /* Initialize */
        std::fill_n(g, size, inf);
        std::fill_n(closed, size, false);
        std::vector<Eigen::VectorXd> path;
        std::priority_queue<std::pair<double, int>> open;

        /* Push the first point into the open set */
        int index = hash(tracker.head(3)); 
        parent[index] = -1; 
        g[index] = time[index] = 0;
        states.col(index) = tracker;
        open.push(std::make_pair(-f(0, tracker.head(3), observation), index));

        /* Main loop */
        while(!open.empty())
        {
            /* Dequeue a state */
            index = open.top().second;
            open.pop();

            /* Check closed and duration */
            if(closed[index])
                continue;
            closed[index] = true;

            /* If found a path */
            if(visible(target, states.col(index).head(3)))
            {
                states.col(index).tail(2).setZero();
                do path.push_back(states.col(index));
                while((index = parent[index]) != -1);
                break;
            }
            
            /* Expand node */
            trigonometric(states(2, index));
            for(tracker[3] = -vm; tracker[3] <= vm; tracker[3] += dv)
            {
                double d = tracker[3] * dt;
                for(tracker[4] = -wm; tracker[4] <= wm; tracker[4] += dw)
                {
                    /* Calculate next state */
                    tracker.head(2) = states.col(index).head(2) + d * trigon;
                    tracker.z() = normalize(states(2, index) + dt * tracker[4]);

                    /* Boundary check */
                    if(2 * std::fabs(tracker.x()) > map->width || 
                       2 * std::fabs(tracker.y()) > map->height) continue;
                    
                    /* Check closed and collision */
                    int idx = hash(tracker.head(3));
                    if(!closed[idx] && map->at(tracker.head(2)) >= radius)
                    {
                        /* If backward, check exploration */
                        if(d < 0 && !map->known(tracker.head(2)))
                            continue;

                        /* Calculate cost */
                        double value = std::fabs(tracker[4]) * radius * dt
                                     + std::fabs(d) + g[index];
                        if(time[index] >= 1 && d < 0)
                            value -= d;
                        
                        /* Update */
                        if(value < g[idx])
                        {
                            g[idx] = value;
                            parent[idx] = index;
                            states.col(idx) = tracker;
                            time[idx] = time[index] + dt;
                            open.push(std::make_pair(-f(
                                value, tracker.head(3), observation
                            ), idx));
                        }
                    }
                }
            }
        }

        /* Convert to Eigen::MatrixXd */
        index = path.size();
        Eigen::MatrixXd matrix(5, index);
        for(Eigen::VectorXd& state: path)
            matrix.col(--index) = state;
        return matrix;
    }

    Eigen::Vector3d HybridAStar::proposal(const Eigen::Vector2d& target,
                                          const Eigen::Vector2d& tracker)
    {
        /* Generate a candidate observation point */
        Eigen::Vector3d observation;
        observation.head(2) = target - ob * (target - tracker).normalized();

        /* Calculate angle */
        int direction = 0;
        double angle = 0.;
        observation.z() = std::atan2(
            observation.y() - tracker.y(),
            observation.x() - tracker.x()
        );
        
        /* Check occlusion */
        if(occlusion(target, observation.head(2)))
            for(angle = deg2rad; angle <= PI; angle += deg2rad)
            {
                trigonometric(observation.z() + angle);
                observation.head(2) = target + ob * trigon;
                if(!occlusion(target, observation.head(2)))
                {
                    direction = 1; break;
                }

                trigonometric(observation.z() - angle);
                observation.head(2) = target + ob * trigon;
                if(!occlusion(target, observation.head(2)))
                {
                    direction = -1; break;
                }
            }
        
        /* Calculate observation yaw angle */
        observation.z() = angle > PI? inf: normalize(
            PI + direction * angle + observation.z()
        );

        return observation;
    }

    bool HybridAStar::occlusion(const Eigen::Vector2d& target,
                                const Eigen::Vector2d& tracker)
    {
        const float d = radius * 2;
        cv::LineIterator line = map->line( 
            map->encode(tracker),
            map->encode(target)
        );
        for(int _ = line.count; _; --_)
            if(map->at(line++.pos()) < d)
                return true;
        return false;
    }

    bool HybridAStar::visible(const Eigen::Vector2d& target, 
                              const Eigen::Vector3d& tracker)
    {
        return 

        /* Check tracking distance */
        std::fabs(ob - (target - tracker.head(2)).norm()) < epsilon &&

        /* Check observation angle */
        std::fabs(normalize(tracker.z() - std::atan2(
            target.y() - tracker.y(), target.x() - tracker.x()
        ))) < 2 * PI / angles &&
        
        /* Check occlusion */
        !occlusion(target, tracker.head(2));
    }
}
