#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

class Parameters {
public:
  int    latency       = 100;  // latency (milliseconds)
  double dt_latency    = 0.1;  // dt for calculating first position because of latency (seconds)
  int    N             = 10;   // number of time steps
  double dt            = 0.05; // time step (seconds)
  double v_target      = 110.; // target velocity (mph)
  double w_cte         = 1.0;  // cost weight for cte^2
  double w_epsi        = 1.0;  // cost weight for epsi^2
  double w_delta_v     = 1e-4; // cost weight for (v - v_target)^2
  double w_steer       = 100.; // cost weight for delta^2
  double w_acc         = 0.;   // cost weight for a^2
  double w_steer_delta = 500;  // cost weight for (delta_t - delta_{t-1})^2
  double w_acc_delta   = 1e-2; // cost weight for (a_t - a_{t-1})^2
  int    control_idx   = 1;    // which control to use from control vector 

  Parameters(std::string path = "");
};

class MPC {
 public:
  Parameters p_;
  MPC(const Parameters& params);

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  std::vector<double> Solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs);
};

#endif /* MPC_H */
