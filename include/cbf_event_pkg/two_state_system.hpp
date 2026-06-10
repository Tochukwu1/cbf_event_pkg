#ifndef CBF_EVENT_PKG__TWO_STATE_SYSTEM_HPP_
#define CBF_EVENT_PKG__TWO_STATE_SYSTEM_HPP_


#include <vector>
#include <Eigen/Dense>

// simulation result
struct SimResult {
    std::vector<double> t;
    std::vector<Eigen::Vector2d> x;
    std::vector<double> h;
    std::vector<int> T;
    std::vector<double> tu;
    std::vector<double> u;
};

// System Parameters
struct  Params
{
    int m{1};
    int n{2};

    double hmin{0.05};
    double c{1.0};
    double eps{8.0};

    Eigen::Vector2d z{1.0, 2.0};
    double r{0.5};

    double alpha_gain{1.0};
    double alpha_p_gain;   // computed later

    double u_min{-40.0};
    double u_max{40.0};

    double event_dt{1e-3};
    double event_tol{1e-8};
};

class TwoStateSystem {
    public:
    Params params;
    TwoStateSystem();

    // Dynamics 
    Eigen::Vector2d driftDynamics(const Eigen::Vector2d& x) const;
    Eigen::Vector2d controlEffectiveness(const Eigen::Vector2d& x) const;
    Eigen::Vector2d systemDynamics(const Eigen::Vector2d& x, double u) const;
    double uDesired(const Eigen::Vector2d& x) const;
    double hBarrier(const Eigen::Vector2d& x) const;
    Eigen::Vector2d hGradient(const Eigen::Vector2d& x) const;
    double lgH(const Eigen::Vector2d& x) const;
    double lfH(const Eigen::Vector2d& x) const;
    double hDotBarrier(const Eigen::Vector2d& x, double u) const;
    double alpha(double h) const;
    double alphaPrime(double h) const;

    // Controllers 
    double cbfQpNom(const Eigen::Vector2d& x) const;
    double adjustedControl(const Eigen::Vector2d& x, double eps_cur) const;

    // Event Triggger Logic
    double constViolation(const Eigen::Vector2d& x, double u_cur) const;
    bool triggerCheck(double t, const Eigen::Vector2d& x, double u_held, double epsilon) const;

    // RK4 Integrator & Simulation
    Eigen::Vector2d rk4Step(const Eigen::Vector2d& x, double u, double dt) const;
    SimResult simulateEventTriggered(Eigen::Vector2d x0, double t0, double tf, double eps_cur, double freq) const; 

};

#endif // CBF_EVENT_PKG__TWO_STATE_SYSTEM_HPP_