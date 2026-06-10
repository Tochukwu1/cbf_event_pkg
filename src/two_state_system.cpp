#include "cbf_event_pkg/two_state_system.hpp"
#include <cmath>
#include <algorithm>

TwoStateSystem::TwoStateSystem()
{
    params.alpha_p_gain = 1.0 + params.c;
}

// Dynamics

Eigen::Vector2d TwoStateSystem::driftDynamics(const Eigen::Vector2d& x) const
{
    double c2x { std::cos(2.0 * x(0)) + 2.0 };
    return Eigen::Vector2d(
        -x(0) + x(1),
        -0.5 * x(0) - 0.5 * x(1) * (1.0 - c2x * c2x)
    );
}

Eigen::Vector2d TwoStateSystem::controlEffectiveness(const Eigen::Vector2d& x) const
{
    return Eigen::Vector2d(0.0, std::cos(2.0 * x(0)) + 2.0);
}

Eigen::Vector2d TwoStateSystem::systemDynamics(const Eigen::Vector2d& x, double u) const
{
    return driftDynamics(x) + controlEffectiveness(x) * u;
}

// Nomimal Control

double TwoStateSystem::uDesired(const Eigen::Vector2d& x) const
{
    return -(std::cos(2.0 * x(0)) + 2.0) * x(1);

}

// Barrier Function

double TwoStateSystem::hBarrier(const Eigen::Vector2d& x) const
{
    return (x - params.z).norm() - params.r;
}


Eigen::Vector2d TwoStateSystem::hGradient(const Eigen::Vector2d& x) const
{
    Eigen::Vector2d d = x - params.z;
    double norm_d = d.norm();

    if (norm_d < 1e-10)
    {
        return Eigen::Vector2d(0.0, 0.0);
    }

    return d / norm_d;
}

// Lie Derivatives

double TwoStateSystem::lgH(const Eigen::Vector2d& x) const
{
    return hGradient(x).dot(controlEffectiveness(x));
}

double TwoStateSystem::lfH(const Eigen::Vector2d& x) const
{
    return hGradient(x).dot(driftDynamics(x));
}

double TwoStateSystem::hDotBarrier(const Eigen::Vector2d& x, double u) const
{
    return lfH(x) + lgH(x) *u;
}

// Class K

double TwoStateSystem::alpha(double h) const
{
    return params.alpha_gain * h;
}

double TwoStateSystem::alphaPrime(double h) const
{
    return params.alpha_p_gain * alpha(h);
}

// CBF-QP

double TwoStateSystem::cbfQpNom(const Eigen::Vector2d& x) const
{
    double u_des { uDesired(x) };
    double h  { hBarrier(x) };
    double L_fh { lfH(x) };
    double L_gh {  lgH(x) };

    double a { L_gh };
    double b  { -alpha(h) - L_fh };

    double u_low { params.u_min };
    double u_high { params.u_max };

    if (a > 1e-8)
    {
        u_low = std::max(u_low, b / a);
    }
    else if (a < 1e-8)
    {
        u_high = std::min(u_high, b / a);
    }

    if (u_low > u_high) // just a check to make sure I didnt do something dumb
    {
        return std::clamp(u_des, params.u_min, params.u_max);
    }

    return std::clamp(u_des, u_low, u_high);
    
}

double TwoStateSystem::adjustedControl(const Eigen::Vector2d& x, double eps_cur) const
{
    double u_nom { cbfQpNom(x) };
    double u = u_nom + (1.0 / eps_cur) * lgH(x);

    return std::clamp(u, params.u_min, params.u_max);
}

// Event Triggger Logic
double TwoStateSystem::constViolation(const Eigen::Vector2d& x, double u_cur) const
{
    double h { hBarrier(x) };
    double L_fh { lfH(x) };
    double L_gh { lgH(x) };

    double a { L_gh };
    double b  { -alpha(h) - L_fh };

    double diff { b - a * u_cur };

    return std::max(0.0, diff);
}

bool TwoStateSystem::triggerCheck(double t, const Eigen::Vector2d& x, double u_held, double epsilon) const
{
    double c_viol { constViolation(x, u_held) };
    return (c_viol >= epsilon);
}

// RK4 Integrator 
Eigen::Vector2d TwoStateSystem::rk4Step(const Eigen::Vector2d& x, double u, double dt) const
{
    Eigen::Vector2d k1 { systemDynamics(x, u) };
    Eigen::Vector2d k2 { systemDynamics(x + 0.5 * dt * k1, u) };
    Eigen::Vector2d k3 { systemDynamics(x + 0.5 * dt * k2, u) };
    Eigen::Vector2d k4 { systemDynamics(x + dt * k3, u) };

    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

// Simulation

SimResult TwoStateSystem::simulateEventTriggered(Eigen::Vector2d x0, double t0, double tf, double eps_cur, double freq) const 
{
    SimResult sim;

    double dt { 1.0 / freq };
    double t { t0 };
    Eigen::Vector2d x { x0 };

    double u { adjustedControl(x, eps_cur) };

    sim.t.push_back(t);
    sim.x.push_back(x);
    sim.h.push_back(hBarrier(x));
    sim.T.push_back(0);
    sim.tu.push_back(t);
    sim.u.push_back(u);

    double max_sub_dt { 1e-3 };

    while (t < tf - 1e-12)
    {
        double t_next { std::min(t + dt, tf) };

        double t_curr = t;
        while (t_curr < t_next - 1e-12)
        {
            double delta_t { std::min(max_sub_dt, t_next - t_curr) };
            x = rk4Step(x, u, delta_t);
            t_curr += delta_t;
        }

        t = t_next;

        sim.t.push_back(t);
        sim.x.push_back(x);
        sim.h.push_back(hBarrier(x));

        bool isterminal = triggerCheck(t, x, u, eps_cur);

        if (isterminal)
        {
            u = adjustedControl(x, eps_cur);

            sim.tu.push_back(t);
            sim.u.push_back(u);
            sim.T.push_back(1);
        }
        else
        {
            sim.T.push_back(0);
        }
    }

    return sim;
}