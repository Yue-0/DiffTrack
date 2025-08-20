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
                             double range, double r, double t, 
                             double max_vel, double max_omega,
                             double distance, double e, double tau)
    :   map(world),
        bezier(prediction),
        angles(angle), radius(r),
        dt(t), vm(max_vel), wm(max_omega),
        dv(max_vel / num), dw(max_omega / num),
        ob(distance), epsilon(e), duration(tau), reacquisition(-1)
    {
        /* For A* search */
        size = angle
             * std::round(world->width / world->resolution)
             * std::round(world->height / world->resolution);
        states.resize(5, size);
        closed = new bool[size];
        parent = new int[size];
        time = new double[size];
        g = new double[size];

        /* For target reacquisition */
        int height = std::ceil(world->height / range);
        int width = std::ceil(world->width / range);
        double y0 = world->offset.height / height;
        double x0 = world->offset.width / width;
        for(int y = 0; y < height; y++)
            for(int x = 0; x < width; x++)
                points.emplace_back(x0 * (x * 2 + 1), y0 * (y * 2 + 1));
        len = points.size();
    }

    Eigen::MatrixXd HybridAStar::reacquire(bool replan,
                                           const Eigen::VectorXd& tracker)
    {
        /* Calculate the observation point */
        Eigen::Vector3d goal;
        if(replan)
            --reacquisition;
        if(reacquisition < 0)
        {
            reacquisition = 0;
            goal.head(2) = bezier->derivative(bezier->duration);
            goal.z() = std::atan2(goal.y(), goal.x());
            goal.head(2) = bfs(bezier->get(bezier->duration));
        }
        else
        {
            if(!reacquisition)
                len = dijkstra(map->encode(bfs(bezier->get(bezier->duration))));
            goal.head(2) = bfs(map->decode(points[reacquisition++ % len]));
            goal.z() = std::atan2(
                goal.y() - tracker.y(), 
                goal.x() - tracker.x()
            );
            if(reacquisition == len << 1)
                reacquisition = len;
        }

        /* Search the trajectory */
        Eigen::MatrixXd trajectory = search(tracker, nan, goal);
        if(trajectory.cols())
            trajectory.col(0) = tracker;
        else
            trajectory = tracker;
        return trajectory;
    }

    bool HybridAStar::plan(Eigen::MatrixXd& trajectory,
                           const Eigen::VectorXd& tracker)
    {
        /* Get predicted position of the target */
        Eigen::Vector2d prediction = bezier->get(bezier->duration);

        /* Check replan */
        reacquisition = -1;
        int n = bezier->n / 2 + 1;
        if(visible(prediction, trajectory.topRightCorner(3, 1)))
            return false;
        
        /* Try to plan */
        while(n--)
        {
            /* Generate a proposal observation point */
            prediction = bezier->get(n * bezier->time);
            Eigen::Vector3d goal = proposal(prediction, tracker.head(2));
            
            /* Path search */
            if(!std::isinf(goal.z()))
            {
                Eigen::MatrixXd path = search(tracker, prediction, goal);
                if(path.cols())
                {
                    path.col(0) = tracker;
                    trajectory = path;
                    return true;
                }
            }
        }

        /* Planning failed */
        return false;
    }

    int HybridAStar::dijkstra(const cv::Point& start)
    {
        /* Initialize */
        int n = 0;
        cv::Size sz = map->offset * 2;
        const int num = points.size();
        std::fill_n(g, sz.area(), inf);
        std::fill_n(closed, sz.area(), false);
        std::priority_queue<std::pair<double, int>> open;

        /* Push the start point into the queue */
        int index = map->encode(start);
        open.push(std::make_pair(g[index] = 0, index));

        /* Main loop */
        while(!open.empty())
        {
            /* Dequeue a point */
            index = open.top().second; open.pop();
            cv::Point point = map->decode(index);
            closed[index] = true;

            /* Check the point */
            for(int p = n; p < num; p++)
                if(points[p] == point)
                {
                    points[p] = points[n];
                    points[n] = point;
                    if(++n == num)
                        return n;
                    break;
                }

            /* Expand the point */
            for(cv::Point& neighbor: expand(sz, point))
            {
                if(map->at(neighbor) <= radius)
                    continue;
                double distance = g[index] + (
                    (point - neighbor).dot(point - neighbor) == 1? 1: sqr
                );
                index = map->encode(neighbor);
                if(distance < g[index])
                    open.push(std::make_pair(-(g[index] = distance), index));
            }
        }

        return n;
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
        index = map->encode(point);
        closed[index] = true;
        queue.push(index);

        /* BFS */
        while(!queue.empty())
        {
            /* Dequeue a point */
            index = queue.front(); queue.pop();
            cv::Point p = map->decode(index);

            /* Expand */
            for(cv::Point& neighbor: expand(sz, p))
            {
                index = map->encode(neighbor);
                if(!closed[index])
                {
                    if(map->at(neighbor) > radius)
                        return map->decode(neighbor);
                    queue.push(index);
                    closed[index] = true;
                }
            }
        }
        return start;
    }

    Eigen::MatrixXd HybridAStar::search(Eigen::VectorXd tracker,
                                        const Eigen::Vector2d& target,
                                        const Eigen::Vector3d& observation)
    {
        /* Initialize */
        const bool vis = !target.hasNaN();

        /* Find a safe start point */
        std::vector<Eigen::VectorXd> path = escape(tracker);
        if(!path.empty()) tracker = path.front();
        int index = hash(tracker.head(3));

        /* Initialize queue and arrays */
        std::fill_n(g, size, inf);
        std::fill_n(closed, size, false);
        std::priority_queue<std::pair<double, int>> open;

        /* Push the first point into the open set */
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
            if(
                vis
                ? visible(target, states.col(index).head(3))
                : arrival(observation.head(2), states.col(index).head(2))
            ){
                do path.push_back(states.col(index));
                while((index = parent[index]) != -1);
                break;
            }
            
            /* Expand node */
            for(Eigen::VectorXd& neighbor: expand(states.col(index)))
            {
                /* Check closed and collision */
                int idx = hash(neighbor.head(3));
                if(!closed[idx] && map->at(neighbor.head(2)) > radius)
                {
                    /* Calculate cost */
                    double value = std::fabs(neighbor[4]) * radius * dt
                                 + std::fabs(neighbor[3]) * dt + g[index];
                    if(neighbor[3] < 0 && 2 * bezier->duration < time[index])
                        value -= neighbor[3] * dt;
                    
                    /* Update */
                    if(value < g[idx])
                    {
                        g[idx] = value;
                        parent[idx] = index;
                        states.col(idx) = neighbor;
                        time[idx] = time[index] + dt;
                        open.push(std::make_pair(-f(
                            value, neighbor.head(3), observation
                        ), idx));
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

    std::vector<Eigen::VectorXd> HybridAStar::escape(Eigen::VectorXd state)
    {
        /* Check obstacle */
        std::vector<Eigen::VectorXd> path;
        if((*g = map->at(state.head(2))) > radius)
            return path;
        
        /* Initialize queue and arrays */
        std::queue<int> queue;
        std::fill_n(closed, size, false);

        /* Push the first point into the queue */
        int idx = hash(state.head(3));
        states.col(idx) = state;
        closed[idx] = true;
        parent[idx] = -1;
        queue.push(idx);
        g[idx] = *g;

        /* BFS */
        while(!queue.empty() && path.empty())
        {
            /* Dequeue a point */
            int index = queue.front(); queue.pop();

            /* Expand */
            for(Eigen::VectorXd& neighbor: expand(states.col(index)))
                if(!closed[idx = hash(neighbor.head(3))])
                {
                    parent[idx] = index;
                    states.col(idx) = neighbor;
                    g[idx] = map->at(neighbor.head(2));
                    
                    if(g[idx] > radius)
                    {
                        do path.push_back(states.col(idx));
                        while((idx = parent[idx]) != -1);
                        break;
                    }
                    else if(g[idx] >= g[index])
                        queue.push(idx);
                    
                    closed[idx] = true;
                }
        }
        return path;
    }

    std::vector<Eigen::VectorXd> HybridAStar::expand(const Eigen::VectorXd& s)
    {
        trigonometric(s.z());
        Eigen::VectorXd state = s;
        std::vector<Eigen::VectorXd> neighbors;
        for(state[3] = vm; state[3] >= -vm; state[3] -= dv)
        {
            double d = state[3] * dt;
            for(state[4] = -wm; state[4] <= wm; state[4] += dw)
            {
                /* Calculate next state */
                state.head(2) = s.head(2) + d * trigon;
                state.z() = normalize(s.z() + dt * state[4]);

                /* Boundary check */
                if(2 * std::fabs(state.x()) > map->width || 
                   2 * std::fabs(state.y()) > map->height) continue;

                /* Check backward */
                if(d < 0 && !map->known(state.head(2))) 
                    continue;
                
                /* Add to neighbors */
                neighbors.push_back(state);
            }
        }
        return neighbors;
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

    std::vector<cv::Point> HybridAStar::expand(const cv::Size& bound,
                                               const cv::Point& point)
    {
        std::vector<cv::Point> neighbors;
        for(int dx = -1; dx <= 1; dx++)
        {
            int x = point.x + dx;
            if(x < 0 || x >= bound.width)
                continue;
            for(int dy = -1; dy <= 1; dy++)
            {
                int y = point.y + dy;
                if(y < 0 || y >= bound.height)
                    continue;
                neighbors.emplace_back(x, y);
            }
        }
        return neighbors;
    }

    bool HybridAStar::arrival(const Eigen::Vector2d& target,
                              const Eigen::Vector2d& tracker)
    {
        return (target - tracker).squaredNorm() < epsilon * epsilon;
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
