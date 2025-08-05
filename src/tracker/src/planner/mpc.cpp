/* @author: YueLin */

#include "planner/mpc.hpp"

namespace diff_track
{
    ModelPredictiveControl::ModelPredictiveControl(
        /* Map and Bezier curve */
        Map* world, Bezier* b,

        /* Robot settings */
        double sz, double t, double od,
        
        /* Weights of cost terms */
        double lambda_a, double lambda_d, double lambda_o,
        double lambda_c, double lambda_f, double lambda_s, 

        /* Motion constraints */
        double max_vel, double max_acc, double max_omega, double max_alpha,

        /* Hyperparameters for L-BFGS */
        int pst, int m, int samples, double e, double step, double d, double ep,

        /* Hyperparameters for Lewis-Overton line search */
        int iters, double wlf, double amj
    ):
        map(world), bezier(b), 
        radius(sz), ob(od), dt(t), kappa(1. / samples), 
        vel2(max_vel * max_vel), acc2(max_acc * max_acc),
        omega2(max_omega * max_omega), alpha2(max_alpha * max_alpha),
        past(pst), mem(m), eps(e), steps(step), delta(d), epsilon(ep),
        iterations(iters), wolfe(wlf), armijo(amj)
    {
        /* Weight of cost terms */
        lambda[A] = lambda_a;
        lambda[D] = lambda_d;
        lambda[O] = lambda_o;
        lambda[C] = lambda_c;
        lambda[F] = lambda_f;
        lambda[S] = lambda_s;

        /* Initialize costs */
        costs = Eigen::VectorXd::Zero(6);

        /* Initialize states */
        n = std::round(b->duration / t);
        trajectory.resize(5, n + 1);
        pos.resize(3, n + 1);
        vel.resize(2, n + 1);
        acc.resize(2, n);
        sin.resize(n);
        cos.resize(n);

        /* Initialize gradients */
        gradient.resize(3, n);
        gx2v = Eigen::MatrixXd::Zero(n, n);
        gy2v = Eigen::MatrixXd::Zero(n, n);
        gx2w = Eigen::MatrixXd::Zero(n, n);
        gy2w = Eigen::MatrixXd::Zero(n, n);
        gz2w = Eigen::MatrixXd::Zero(n, n);
        gz2w.triangularView<Eigen::Lower>().setConstant(t);

        /* Initialize vectors for L-BFGS */
        n <<= 1;
        x.resize(n);
        g.resize(n);
        x0.resize(n);
        g0.resize(n);

        /* Allocate memory for L-BFGS */
        pf = Eigen::VectorXd::Zero(pst);
        dx = Eigen::MatrixXd::Zero(n, m);
        dg = Eigen::MatrixXd::Zero(n, m);
        limit = Eigen::VectorXd::Zero(m);
        memory = Eigen::VectorXd::Zero(m);
        n >>= 1;
    }

    Eigen::MatrixXd ModelPredictiveControl::control(const Eigen::VectorXd& now,
                                                    const Eigen::MatrixXd& ref)
    {
        /* Initialize velocity */
        if(n < ref.cols())
            vel = ref.bottomLeftCorner(2, n + 1);
        else
        {
            vel.leftCols(ref.cols()) = ref.bottomRows(2);
            vel.rightCols(n - ref.cols() + 1).setZero();
        }
        
        /* Initialize initial state */
        pos.col(0) = now.head(3);
        vel.col(0) = now.tail(2);

        /* Optimization */
        x.head(n) = vel.topRightCorner(1, n).transpose();
        x.tail(n) = vel.bottomRightCorner(1, n).transpose();
        optimize();
        propagate();

        // std::cout << "Costs:"
        //           << "\nJa: " << costs[A]
        //           << "\nJd: " << costs[D]
        //           << "\nJo: " << costs[O]
        //           << "\nJc: " << costs[C]
        //           << "\nJf: " << costs[F]
        //           << "\nJs: " << costs[S]
        //           << std::endl;

        /* Get trajectory */
        trajectory.topRows(3) = pos;
        trajectory.bottomRows(2) = vel;
        return trajectory;
    }

    double ModelPredictiveControl::cost()
    {
        /* Initialize */
        costs.setZero();
        gradient.setZero();
        double j = dt * dt;
        Eigen::Vector2d grad, target;

        /* Propagate states */
        propagate();
        acc = (vel.rightCols(n) - vel.leftCols(n)) / dt;
        
        /* Initialize gradients of states w.r.t. control inputs */
        gx2v.triangularView<Eigen::Lower>() = (dt * cos)
                                            . transpose()
                                            . replicate(n, 1);
        gy2v.triangularView<Eigen::Lower>() = (dt * sin)
                                            . transpose()
                                            . replicate(n, 1);
        for(int i = 1; i < n; i++)
        {
            gx2w.row(i).head(i) = gx2w.row(i - 1).head(i) 
                                - j * vel(0, i + 1) * sin.head(i).transpose();
            gy2w.row(i).head(i) = gy2w.row(i - 1).head(i) 
                                + j * vel(0, i + 1) * cos.head(i).transpose();
        }

        /* Visibility costs */
        for(int t = 1; t <= n; t++)
        {
            /* Occlusion cost */
            target = bezier->get(dt * t + time);
            for(double k = kappa; k <= 1; k += kappa)
            {
                j = 2 * radius - map->esdf(
                    k * pos.col(t).head(2) + (1 - k) * target, grad
                );
                if(j > 0)
                {
                    costs[C] += j;
                    costs.tail(2) -= k * grad;
                }
            }
            gradient.col(t - 1).head(2) += 2
                                         * costs[C]
                                         * lambda[O]  
                                         * costs.tail(2);
            costs[O] += lambda[O] * costs[C] * costs[C];
            costs.tail(3).setZero();

            /* Tracking distance cost */
            target -= pos.col(t).head(2);
            costs[D] += lambda[D] * td(target, grad);
            gradient.col(t - 1).head(2) += lambda[D] * grad;

            /* Calculate angle cost */
            j = pos(2, t) - std::atan2(target.y(), target.x());
            costs[A] += lambda[A] * (1 - std::cos(j));

            /* Calculate gradient of angle cost w.r.t. yaw angle */
            j = lambda[A] * std::sin(j);
            gradient(2, t - 1) += j;

            /* Calculate gradient of angle cost w.r.t. position */
            j /= target.squaredNorm();
            gradient(0, t - 1) += j * target.y();
            gradient(1, t - 1) -= j * target.x();
        }
        
        /* Calculate collision cost and its gradient */
        for(int t = 0; t < n; t++)
            if((j = radius - map->esdf(pos.col(t + 1).head(2), grad)) > 0)
            {
                costs[C] += lambda[C] * j * j;
                gradient.col(t).head(2) -= 2 * lambda[C] * j * grad;
            }

        /* Calculate smoothness cost */
        Eigen::Matrix3Xd dp3 = pos.middleCols(1, n - 2) * 3
                             - pos.middleCols(2, n - 2) * 3
                             + pos.rightCols(n - 2)
                             - pos.leftCols(n - 2);
        costs[S] = lambda[S] * dp3.squaredNorm();
        
        /* Calculate gradient of smoothness cost */
        dp3 *= 2 * lambda[S];
        gradient.leftCols(n - 2) += 3 * dp3;
        gradient.leftCols(n - 3) -= dp3.rightCols(n - 3);
        gradient.middleCols(1, n - 2) -= 3 * dp3;
        gradient.rightCols(n - 2) += dp3;

        /* Propagate gradients */
        g.head(n) = (gradient.row(0) * gx2v).transpose()
                  + (gradient.row(1) * gy2v).transpose();
        g.tail(n) = (gradient.row(0) * gx2w).transpose()
                  + (gradient.row(1) * gy2w).transpose()
                  + (gradient.row(2) * gz2w).transpose();

        /* Velocity cost */
        for(int t = 1; t < n; t++)
        {
            if((j = vel(0, t) * vel(0, t) - vel2) > 0)
            {
                costs[F] += lambda[F] * j;
                g[t - 1] += 2 * lambda[F] * vel(0, t);
            }
            if((j = vel(1, t) * vel(1, t) - omega2) > 0)
            {
                costs[F] += lambda[F] * j;
                g[n + t - 1] += 2 * lambda[F] * vel(1, t);
            }
        }

        /* Acceleration cost */
        for(int t = 1; t < n - 1; t++)
        {
            if((j = acc(0, t) * acc(0, t) - acc2) > 0)
            {
                costs[F] += lambda[F] * j;
                j = 2 * lambda[F] * acc(0, t) / dt;
                g[t - 1] -= j;
                g[t] += j;
            }
            if((j = acc(1, t) * acc(1, t) - alpha2) > 0)
            {
                costs[F] += lambda[F] * j;
                j = 2 * lambda[F] * acc(1, t) / dt;
                g[n + t - 1] -= j;
                g[n + t] += j;
            }
        }

        /* Sum all costs */
        return costs.sum();
    }

    void ModelPredictiveControl::propagate()
    {
        vel.topRightCorner(1, n) = x.head(n).transpose();
        vel.bottomRightCorner(1, n) = x.tail(n).transpose();
        for(int t = 0; t < n; t++)
        {
            sin[t] = std::sin(pos(2, t));
            cos[t] = std::cos(pos(2, t));
            pos(2, t + 1) = pos(2, t) + dt * vel(1, t + 1);
            pos(1, t + 1) = pos(1, t) + dt * vel(0, t + 1) * sin[t];
            pos(0, t + 1) = pos(0, t) + dt * vel(0, t + 1) * cos[t];
        }
    }

    double ModelPredictiveControl::optimize()
    {
        /* Initialize the limited memory */
        pf.setZero();
        dx.setZero();
        dg.setZero();
        limit.setZero();
        memory.setZero();

        /* Evaluate the function value and its gradient */
        double fx = pf[0] = cost(); 

        if(!convergance())
        {
            int iter = 1, end = 0, b = 0;
            double step = 1. / g.norm();
            Eigen::VectorXd d = -g;
            while(true)
            {
                /* Store the current position and gradient vectors */
                x0 = x; g0 = g;

                /* Lewis-Overton line search */
                if(step >= steps) step = steps * 0.5;
                if(!search(&fx, &step, d))
                {
                    x = x0; g = g0; break;
                }

                /* Convergance test */
                if(convergance())
                    break;

                /* Test for stopping criterion */
                if(iter >= past && 
                   std::fabs(pf[iter % past] - fx) / 
                   std::max(std::fabs(fx), 1.) < delta) break;
                pf[iter++ % past] = fx;

                /* L-BFGS update */
                d = -g;
                dx.col(end) = x - x0;
                dg.col(end) = g - g0;

                /* Cautious update */
                double dg2 = dg.col(end).squaredNorm();
                double dxg = memory[end] = dg.col(end).dot(dx.col(end));
                if(dxg > eps * dx.col(end).squaredNorm() * g0.norm())
                {
                    int i = end = (end + 1) % mem, _;
                    for(b =_= std::min(mem, b + 1); _; --_)
                    {
                        i = (i + mem - 1) % mem;
                        limit[i] = dx.col(i).dot(d) / memory[i];
                        d -= limit[i] * dg.col(i);
                    }
                    for(d *= dxg / dg2 ;_< b; ++_)
                    {
                        d += (
                            limit[i] - dg.col(i).dot(d) / memory[i]
                        ) * dx.col(i);
                        i = (i + 1) % mem;
                    }
                }
                step = 1;
            }
        }
        return fx;
    }

    bool ModelPredictiveControl::convergance() const
    {
        return epsilon >= g.cwiseAbs().maxCoeff() / std::max(
            x.cwiseAbs().maxCoeff(), 1.
        );
    }

    bool ModelPredictiveControl::search(double* value, double* step,
                                        const Eigen::VectorXd& direction)
    {
        /* Compute the initial gradient in the search direction */
        double grad = g0.dot(direction);
        if(grad > 0) return false;
        double a = grad * armijo;
        double w = grad * wolfe;

        /* Line search */
        double min = 0, max = steps, fx = *value;
        bool ac = false, touched = false;
        int iter = 0;
        while(true)
        {
            /* Evaluate the function and gradient values */
            x = x0 + *step * direction; *value = cost();
            if(std::isinf(*value) || std::isnan(*value))
                return false;
            
            /* Check the Armijo condition */
            if(*step * a < *value - fx)
            {
                max = *step; ac = true;
            }
            /* Check the waek Wolfe condition */
            else if(w > g.dot(direction))
                min = *step;
            else
                return true;
            
            /* Maximum number of iteration */
            if(++iter >= iterations) return false;

            /* Relative interval width is at least machine precision */
            if(ac && (max - min) < max * o) return false;

            /* Update step */
            if(ac) *step = (min + max) / 2; else *step *= 2;
            if(*step * steps < 1) return false;
            if(*step > steps)
            {
                if(touched)
                    return false;
                touched = true;
                *step = steps;
            }
        }
    }

    double ModelPredictiveControl::td(const Eigen::Vector2d& d,
                                      Eigen::Vector2d& grad) const
    {
        double distance = d.norm();
        grad = (distance + ob) * (ob - distance) 
             / (distance * distance * distance) * d;
        return (distance - ob) * (distance - ob) / distance;
    }
}
