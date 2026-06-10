#include <iostream>
#include <fstream>
#include "cbf_event_pkg/two_state_system.hpp"

int main()
{
    TwoStateSystem sys;

    double t0 { 0.0 };
    double tf { 18.0 };
    Eigen::Vector2d x0(0.0, 3.0);

    // Event-triggered Simulation (Varying Epsilon)
    std::vector<double> epsilon_vals = {4.0, 3.0, 2.0, 1.0};
    double sample_time_fixed = 1e3; // Hertz

    std::cout << "Running simulations...\n";

    std::ofstream file("simulation_results.csv");
    file << "epsilon,t,x1,x2,h,T\n";

    for (double eps: epsilon_vals)
    {
        std::cout << "Simulating for epsilon = " << eps << "...\n";
        SimResult res = sys.simulateEventTriggered(x0, t0, tf, eps, sample_time_fixed);

        for (std::size_t i = 0; i < res.t.size(); ++i)
        {
            file << eps << ","
                 << res.t[i] << ","
                 << res.x[i](0) << ","
                 << res.x[i](1) << ","
                 << res.h[i] << ","
                 << res.T[i] << "\n";
        }

    }

    file.close();
    std::cout << "Simulations complete. Results saved to 'simulation_results.csv'.\n";

    return 0;
 
}