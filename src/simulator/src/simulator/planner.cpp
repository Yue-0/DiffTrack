/* @author: YueLin */

#include <queue>
#include <utility>

#include "simulator/planner.hpp"

namespace sim
{
    Planner::~Planner()
    {
        delete[] g;
        delete[] parent;
        delete[] closed;
    }

    Planner::Planner(Robot* r, RandomMap* m, double t, double v, double a):
        robot(r), map(m), time(t), vm(v), vm2(v * v), am2(a * a), sample(v * t)
    {
        ds = r->radius * 2;

        /* For A* algorithm */
        rows = m->rows();
        cols = m->cols();
        g = new double[rows * cols];
        parent = new int[rows * cols];
        closed = new bool[rows * cols];

        /* For B-spline parameterization */
        dp << 1, 4, 1; dp /= 6; 
        dv << -1, 0, 1; dv /= 2 * t; 
        da << 1, -2, 1; da /= t * t;

        /* For L-BFGS optimization */
        pf.resize(PAST);
        limit.resize(MEMORY);
        memory.resize(MEMORY);
    }

    void Planner::plan(double xg, double yg)
    {
        /* Path search */
        Eigen::Matrix2Xd path = search(
            tf(xg, yg), tf(robot->pose.x, robot->pose.y)
        );

        /* Trajectory optimization */
        if(path.cols() > 1)
        {
            path(0, 0) = robot->pose.x;
            path(1, 0) = robot->pose.y;
            path(0, path.cols() - 1) = xg;
            path(1, path.cols() - 1) = yg;
            optimize(path = bspline(path));
        }
        
        /* Convert to ROS message */
        message(path);
    }   

    geometry_msgs::msg::Twist Planner::control()
    {
        /* Initialize */
        geometry_msgs::msg::Twist v;
        v.linear.x = v.linear.y = v.angular.z = 0;
        if(msg.poses.empty()) return v;

        /* Get next pose */
        double x = msg.poses.back().pose.position.x;
        double y = msg.poses.back().pose.position.y;
        double yaw = std::atan2(y - robot->pose.y, x - robot->pose.x);
        
        /* Calculate velocity */
        const double wm = vm * SQR2;
        v.angular.z = normalize(yaw - robot->pose.yaw) / time;
        if(std::fabs(v.angular.z) >= wm)
            v.angular.z = std::copysign(wm, v.angular.z);
        else
        {
            double sin, cos, det;
            double dx = x - robot->pose.x;
            double dy = y - robot->pose.y;
            if(std::fabs(v.angular.z) < 1e-2)
            {
                det = 1. / time;
                sin = std::sin(robot->pose.yaw);
                cos = std::cos(robot->pose.yaw);
            }
            else
            {
                sin = (std::cos(robot->pose.yaw) - std::cos(yaw)) / v.angular.z;
                cos = (std::sin(yaw) - std::sin(robot->pose.yaw)) / v.angular.z;
                det = 1. / (sin * sin + cos * cos);
            }
            v.linear.x = (dx * cos + dy * sin) * det;
            v.linear.y = (dy * cos - dx * sin) * det;

            /* Check feasibility */
            bool feasible = true;
            if(std::fabs(v.linear.x) > vm)
            {
                feasible = false;
                v.linear.y *= std::fabs(vm / v.linear.x);
                v.linear.x = std::copysign(vm, v.linear.x);
            }
            if(std::fabs(v.linear.y) > vm)
            {
                feasible = false;
                v.linear.x *= std::fabs(vm / v.linear.y);
                v.linear.y = std::copysign(vm, v.linear.y);
            }
            if(feasible)
                msg.poses.pop_back();
        }
        return v;
    }

    cv::Point Planner::bfs(const cv::Point& start)
    {
        cv::Point point = start;
        
        /* Initialize queue and arrays */
        std::queue<int> open;
        int idx = cols * rows;
        std::fill_n(closed, idx, false);

        /* Push the first point into the queue */
        int index = encode(point);
        closed[index] = true;
        open.push(index);

        /* BFS */
        while(!open.empty())
        {
            /* Dequeue a point */
            index = open.front();
            point = decode(index);
            open.pop();

            /* Expand the point */
            for(cv::Point& neighbor: expand(point))
                if(!closed[idx = encode(neighbor)])
                {
                    if(map->index(neighbor) > ds)
                        return neighbor;
                    closed[idx] = true;
                    open.push(idx);
                }
        }
        return start;
    }

    cv::Point Planner::decode(int code) const
    {
        cv::Point point;
        point.y = code / cols;
        point.x = code - cols * point.y;
        return point;
    }

    cv::Point Planner::tf(double x, double y) const
    {
        return cv::Point(
            std::round(x / map->resolution) + cols / 2, 
            std::round(y / map->resolution) + rows / 2
        );
    }

    Eigen::Vector2d Planner::tf(const cv::Point& point) const
    {
        return Eigen::Vector2d(
            (point.x - cols / 2) * map->resolution, 
            (point.y - rows / 2) * map->resolution
        );
    }

    int Planner::encode(const cv::Point& point) const
    {
        return point.x + point.y * cols;
    }

    std::vector<int> Planner::dijkstra(const Eigen::MatrixXd& graph)
    {
        std::vector<int> path;
        const int n = graph.rows();

        /* Initialize arrays and queue */
        std::fill_n(g, n, INF);
        std::fill_n(parent, n, -1);
        std::fill_n(closed, n, false);
        std::priority_queue<std::pair<double, int>> open;

        /* Push the first node into the queue */
        open.push(std::make_pair(g[0] = 0, 0));

        /* Dijkstra's algorithm */
        while(!open.empty())
        {
            /* Dequeue a node */
            int u = open.top().second; 
            closed[u] = true;
            open.pop();

            /* If found a path */
            if(u == n - 1)
            {
                do path.push_back(u);
                while((u = parent[u]) != -1);
                break;
            }

            /* Expand the node */
            for(int v = u + 1; v < n; v++)
                if(!closed[v] && graph(u, v) > 0)
                {
                    double distance = g[u] + graph(u, v);
                    if(distance < g[v])
                    {
                        parent[v] = u;
                        g[v] = distance;
                        open.push(std::make_pair(-distance, v));
                    }
                }
        }
        return path;
    }

    std::vector<cv::Point> Planner::astar(cv::Point start, cv::Point goal)
    {
        std::vector<cv::Point> path;

        /* Boundaries */
        goal.x = std::max(std::min(goal.x, cols - 1), 0);
        goal.y = std::max(std::min(goal.y, rows - 1), 0);
        start.x = std::max(std::min(start.x, cols - 1), 0);
        start.y = std::max(std::min(start.y, rows - 1), 0);

        /* Obstacle check */
        if(map->index(goal) < ds) goal = bfs(goal);
        if(map->index(start) < ds) start = bfs(start);

        /* Initialize arrays and queue */
        int idx = cols * rows;
        std::fill_n(g, idx, INF);
        std::fill_n(parent, idx, -1);
        std::fill_n(closed, idx, false);
        std::priority_queue<std::pair<double, int>> open;
        
        /* Push the first point into the queue */
        int index = encode(start); g[index] = 0;
        open.push(std::make_pair(-f(index, start, goal), index));

        /* A* algorithm */
        while(!open.empty())
        {
            /* Dequeue a point */
            index = open.top().second;
            start = decode(index);
            open.pop();

            /* Check visited */
            if(closed[index])
                continue;
            closed[index] = true;

            /* If found a path */
            if(start == goal)
            {
                path.push_back(goal);
                while((index = parent[index]) != -1)
                    path.push_back(decode(index));
                break;
            }

            /* Expand the point */
            for(cv::Point& point: expand(start))
                if(map->index(point) > ds && !closed[idx = encode(point)])
                {
                    double value = f(index, start, point);
                    if(value < g[idx])
                    {
                        g[idx] = value;
                        parent[idx] = index;
                        open.push(std::make_pair(-f(idx, point, goal), idx));
                    }
                }
        }
        return path;
    }

    Eigen::Matrix2Xd Planner::search(const cv::Point& goal,
                                     const cv::Point& start)
    {
        /* Run A* algorithm */
        std::vector<cv::Point> path = astar(start, goal);
        int n = path.size(); if(n <= 1) return Eigen::Matrix2Xd::Zero(2, 0);

        /* Select keypoints of the path */
        std::vector<int> keypoints;
        keypoints.push_back(0);
        if(n > 3)
        {
            cv::Point delta[2] = {path[1] - path[0], path[2] - path[1]};
            for(int p = 2; ++p <= n; delta[p & 1] = path[p] - path[p - 1])
                if(delta[0].x * delta[1].y - delta[0].y * delta[1].x)
                    keypoints.push_back(p - 2);
        }
        keypoints.push_back(n - 1);

        /* Construct graph */
        n = keypoints.size();
        Eigen::MatrixXd graph = -Eigen::MatrixXd::Ones(n, n);
        for(int u = 0; u < n; u++)
            for(int v = u + 1; v < n; v++)
            {
                bool connected = true;
                cv::LineIterator line = map->line(
                    path[keypoints[u]], path[keypoints[v]]
                );
                for(int _ = line.count; connected && _; --_)
                    if(map->index(line++.pos()) < ds)
                        connected = false;
                if(connected)
                    graph(u, v) = std::hypot(
                        path[keypoints[u]].x - path[keypoints[v]].x,
                        path[keypoints[u]].y - path[keypoints[v]].y
                    );
            }

        /* Prune the path */
        std::vector<int> keep = dijkstra(graph);
        n = keep.size();

        /* Sample points evenly spaced along the path */
        int p = 1;
        bool next = true;
        double length = 0;
        Eigen::Vector2d p1, p2;
        std::vector<Eigen::Vector2d> points;
        points.push_back(p1 = tf(path[keypoints[keep[0]]]));
        while(p < n)
        {
            if(next) p2 = tf(path[keypoints[keep[p]]]);
            double delta = (p1 - p2).norm();
            if(length + delta >= sample)
            {
                double lambda = (sample - length) / delta;
                p1 = lambda * p2 + (1 - lambda) * p1;
                points.push_back(p1);
                next = false;
                length = 0;
            }
            else
            {
                length += delta;
                next = true;
                ++p;
            }
        }
        points.push_back(tf(path[keypoints[keep.back()]]));
        return Eigen::Map<Eigen::Matrix2Xd>(
            reinterpret_cast<double*>(points.data()), 2, points.size()
        );
    }

    std::vector<cv::Point> Planner::expand(const cv::Point& point) const
    {
        std::vector<cv::Point> neighbors;
        for(int dx = -1; dx <= 1; dx++)
        {
            int x = point.x + dx;
            if(x < 0 || x >= cols)
                continue;
            for(int dy = -1; dy <= 1; dy++)
            {
                int y = point.y + dy;
                if(y < 0 || y >= rows)
                    continue;
                neighbors.emplace_back(x, y);
            }
        }
        return neighbors;
    }

    double Planner::f(int index, const cv::Point& point, const cv::Point& goal)
    const
    {
        return g[index] + std::hypot(point.x - goal.x, point.y - goal.y);
    }

    Eigen::Matrix2Xd Planner::bspline(const Eigen::Matrix2Xd& path)
    {
        const int n = path.cols();
        Eigen::Matrix2Xd ctrl(2, n + 2);
        Eigen::MatrixX2d points(n + 4, 2);
        Eigen::MatrixXd m = Eigen::MatrixXd::Zero(n + 4, n + 2);

        /* Initialize equations */
        for(int i = 0; i < 3; i++)
            m.diagonal(i).head(n).setConstant(dp[i]);
        points.topRows(n) = path.transpose();
        points.bottomRows(4).setZero();
        m.block(n, 0, 1, 3) = m.block(n + 1, n - 1, 1, 3) = dv.transpose();
        m.block(n + 2, 0, 1, 3) = m.bottomRightCorner(1, 3) = da.transpose();
        
        /* Solve control points */
        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> c = m.colPivHouseholderQr();
        for(int i = 0; i < 2; i++)
            ctrl.row(i) = c.solve(points.col(i)).transpose();
        return ctrl;
    }

    double Planner::cost(const Eigen::VectorXd& var, Eigen::VectorXd& gradient)
    {
        /* Initialize */
        double j, fx = 0;
        gradient.setZero();
        Eigen::Vector2d grad;
        const int n = var.size() >> 1;
        Eigen::Map<const Eigen::Matrix2Xd> pos(var.data(), 2, n);
        Eigen::Map<Eigen::Matrix2Xd> gradients(gradient.data(), 2, n);
        
        /* Calculate velocity */
        int m = n - 1;
        Eigen::Matrix2Xd vel = pos.rightCols(m) - pos.leftCols(m);
        
        /* Calculate acceleration */
        --m;
        Eigen::Matrix2Xd acc = vel.rightCols(m) - vel.leftCols(m);
        
        /* Smoothness cost */
        fx += acc.squaredNorm();
        gradients.leftCols(m) += 2 * acc;
        gradients.rightCols(m) += 2 * acc;
        gradients.middleCols(1, m) -= 4 * acc;

        /* Acceleration cost */
        const double t2 = 1. / (time * time);
        const double t4 = t2 * t2;
        for(int t = 0; t < m; t++)
            if((j = acc.col(t).squaredNorm() * t4 - am2) > 0)
            {
                fx += j;
                grad = 2 * t4 * acc.col(t);
                gradients.col(t + 1) -= 2 * grad;
                gradients.col(t + 2) += grad;
                gradients.col(t) += grad;
            }
        
        /* Velocity cost */
        ++m;
        for(int t = 0; t < m; t++)
            if((j = vel.col(t).squaredNorm() * t2 - vm2) > 0)
            {
                fx += j;
                grad = 2 * t2 * vel.col(t);
                gradients.col(t) -= grad;
                gradients.col(t + 1) += grad;
            }
        
        /* Safety cost */
        m -= 2;
        for(int t = 3; t < m; t++)
        {
            double x = pos(0, t) / map->resolution + cols / 2;
            double y = pos(1, t) / map->resolution + rows / 2;
            x = std::max(std::min(x, cols - 2.), 0.);
            y = std::max(std::min(y, rows - 2.), 0.);
            
            /* Linear interpolation */
            int x1 = x, y1 = y;
            int x2 = x1 + 1, y2 = y1 + 1;
            double u = x - x1, v = y - y1;
            double u_ = 1. - u, v_ = 1. - v;
            j = u * v * map->index(x2, y2)
              + u * v_ * map->index(x2, y1)
              + u_ * v * map->index(x1, y2)
              + u_ * v_ * map->index(x1, y1);
            
            /* Calculate gradient */
            if((j = ds - j) > 0)
            {
                fx += LAMBDA * j * j;
                gradients(0, t) += 2 * LAMBDA * j * (
                    + v * map->index(x1, y2) + v_ * map->index(x1, y1)
                    - v * map->index(x2, y2) - v_ * map->index(x2, y1)
                );
                gradients(1, t) += 2 * LAMBDA * j * (
                    + u * map->index(x2, y1) + u_ * map->index(x1, y1)
                    - u * map->index(x2, y2) - u_ * map->index(x1, y2)
                );
            }
        }

        gradients.leftCols(3).setZero();
        gradients.rightCols(3).setZero();
        return fx;
    }

    double Planner::optimize(Eigen::Matrix2Xd& var)
    {
        /* Prepare intermediate variables */
        const int n = var.size();
        Eigen::VectorXd grad(n), x0(n), g0(n);
        Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd>(var.data(), n);

        /* Initialize the limited memory */
        limit.setZero(); memory.setZero();
        Eigen::MatrixXd dx = Eigen::MatrixXd::Zero(n, MEMORY);
        Eigen::MatrixXd dg = Eigen::MatrixXd::Zero(n, MEMORY);

        /* Evaluate the function value and its gradient */
        double fx = pf[0] = cost(x, grad); 
        Eigen::VectorXd d = -grad;

        if(!convergance(x, grad))
        {
            int iter = 1, end = 0, b = 0;
            double step = 1. / d.norm();
            while(true)
            {
                /* Store the current position and gradient vectors */
                x0 = x; g0 = grad;

                /* Lewis-Overton line search */
                if(step >= STEP) step = STEP * 0.5;
                if(!search(grad, &step, x, &fx, d, x0, g0))
                {
                    x = x0; grad = g0; break;
                }

                /* Convergance test */
                if(convergance(x, grad))
                    break;

                /* Test for stopping criterion */
                if(iter >= PAST && 
                   std::fabs(pf[iter % PAST] - fx) / 
                   std::max(std::fabs(fx), 1.) < DELTA) break;
                pf[iter++ % PAST] = fx;

                /* L-BFGS update */
                d = -grad;
                dx.col(end) = x - x0;
                dg.col(end) = grad - g0;

                /* Cautious update */
                double dg2 = dg.col(end).squaredNorm();
                double dgx = memory[end] = dg.col(end).dot(dx.col(end));
                if(dgx > dx.col(end).squaredNorm() * g0.norm() * EPS)
                {
                    int i = end = (end + 1) % MEMORY, _;
                    for(b = _ = std::min(MEMORY, b + 1); _; --_)
                    {
                        i = (i + MEMORY - 1) % MEMORY;
                        limit[i] = dx.col(i).dot(d) / memory[i];
                        d -= limit[i] * dg.col(i);
                    }
                    for(d *= dgx / dg2 ;_< b; ++_)
                    {
                        d += (
                            limit[i] - dg.col(i).dot(d) / memory[i]
                        ) * dx.col(i);
                        i = (i + 1) % MEMORY;
                    }
                }
                step = 1;
            }
        }
        Eigen::Map<Eigen::VectorXd>(var.data(), n) = x;
        return fx;
    }

    bool Planner::search(Eigen::VectorXd& grad, double* step,
                         Eigen::VectorXd& var, double* value,
                         const Eigen::VectorXd& direction,
                         const Eigen::VectorXd& x0,
                         const Eigen::VectorXd& g0)
    {
        int iter = 0;
        bool ac = false, touched = false;
        double min = 0, max = STEP, fx = *value;

        /* Compute the initial gradient in the search direction */
        double gradient = g0.dot(direction);
        if(gradient > 0) return false;
        const double wolfe = WOLFE * gradient;
        const double armijo = ARMIJO * gradient;

        /* Line search */
        while(true)
        {
            /* Evaluate the function and gradient values */
            *value = cost(var = x0 + *step * direction, grad);
            if(std::isinf(*value) || std::isnan(*value))
                return false;
            
            /* Check the Armijo condition */
            if(*value - fx > *step * armijo)
            {
                max = *step; ac = true;
            }
            /* Check the waek Wolfe condition */
            else if(grad.dot(direction) < wolfe)
                min = *step;
            else
                return true;
            
            /* Maximum number of iteration */
            if(++iter >= ITERATIONS) return false;

            /* Relative interval width is at least machine precision */
            if(ac && (max - min) < max * 1e-16) return false;

            /* Update step */
            if(ac) *step = (min + max) / 2; else *step *= 2;
            if(*step < 1. / STEP) return false;
            if(*step > STEP)
            {
                if(touched) 
                    return false;
                touched = true;
                *step = STEP;
            }
        }
    }

    bool Planner::convergance(const Eigen::VectorXd& var,
                              const Eigen::VectorXd& grad) const
    {
        return EPSILON >= grad.cwiseAbs().maxCoeff() / std::max(
            var.cwiseAbs().maxCoeff(), 1.
        );
    }

    void Planner::message(Eigen::Matrix2Xd path)
    {
        const int n = path.cols() - 2;
        if(n <= 1)
            msg.poses.clear();
        else
        {
            path = (
                path.leftCols(n) + 4 * path.middleCols(1, n) + path.rightCols(n)
            ).eval() / 6.;
            msg.poses.resize(n);
            for(int p = 0; p < n; p++)
            {
                msg.poses[p].pose.position.x = path(0, n - p - 1);
                msg.poses[p].pose.position.y = path(1, n - p - 1);
            }
        }
    }
}
