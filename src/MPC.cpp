#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <fstream>
#include <sstream>

using CppAD::AD;

namespace {
  // TODO: Set the timestep length and duration
  //size_t N = 10;
  //double dt = 0.05;
  //double v_target = 110.;

  int N = 10;
  double dt = 0.05;
  int x_start     = 0;
  int y_start     = x_start + N;
  int psi_start   = y_start + N;
  int v_start     = psi_start + N;
  int cte_start   = v_start + N;
  int epsi_start  = cte_start + N;
  int delta_start = epsi_start + N;
  int a_start     = delta_start + N - 1;

  // This value assumes the model presented in the classroom is used.
  //
  // It was obtained by measuring the radius formed by running the vehicle in the
  // simulator around in a circle with a constant steering angle and velocity on a
  // flat terrain.
  //
  // Lf was tuned until the the radius formed by the simulating the model
  // presented in the classroom matched the previous radius.
  //
  // This is the length from front to CoG that has a similar radius.
  const double Lf = 2.67;
}

Parameters::Parameters(std::string path) {
  if (path == "") path = "../params";
  std::ifstream param_file(path);

  if (!param_file) {
    std::cout << "Using default parameters!\n";
  }
  else {
    std::string line;
    while(std::getline(param_file, line)) {
      std::stringstream line_stream(line);
      std::string field;
      line_stream >> field;
      if (field == "latency") { line_stream >> latency; }
      if (field == "dt_latency") { line_stream >> dt_latency; }
      if (field == "N") { line_stream >> N; }
      if (field == "dt") { line_stream >> dt; }
      if (field == "v_target") { line_stream >> v_target; }
      if (field == "w_cte") { line_stream >> w_cte; }
      if (field == "w_epsi") { line_stream >> w_epsi; }
      if (field == "w_delta_v") { line_stream >> w_delta_v; }
      if (field == "w_steer") { line_stream >> w_steer; }
      if (field == "w_acc") { line_stream >> w_acc; }
      if (field == "w_steer_delta") { line_stream >> w_steer_delta; }
      if (field == "w_acc_delta") { line_stream >> w_acc_delta; }
      if (field == "control_idx") { line_stream >> control_idx; }
    }
    std::cout << "latency: " << Parameters::latency << std::endl;
    std::cout << "dt_latency: " << Parameters::dt_latency << std::endl;
    std::cout << "N: " << N << std::endl;
    std::cout << "dt: " << dt << std::endl;
    std::cout << "v_target: " << v_target << std::endl;
    std::cout << "w_cte: " << w_cte << std::endl;
    std::cout << "w_epsi: " << w_epsi << std::endl;
    std::cout << "w_delta_v: " << w_delta_v << std::endl;
    std::cout << "w_steer: " << w_steer << std::endl;
    std::cout << "w_acc: " << w_acc << std::endl;
    std::cout << "w_steer_delta: " << w_steer_delta << std::endl;
    std::cout << "w_acc_delta: " << w_acc_delta << std::endl;
    std::cout << "control_idx: " << control_idx << std::endl;
  }
}
// Evaluate a polynomial.
AD<double> polyeval(Eigen::VectorXd coeffs, AD<double>& x) {
  AD<double> result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * CppAD::pow(x, i);
  }
  return result;
}

// Evaluate the first derivative of a polynomial.
AD<double> polydereval(Eigen::VectorXd coeffs, AD<double>& x) {
  AD<double> result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    result += coeffs[i] * i * CppAD::pow(x, i-1);
  }
  return result;
}

class FG_eval {
 public:
  // Fitted polynomial coefficients
  const Eigen::VectorXd& coeffs_;
  const Parameters& p_;
  FG_eval(const Eigen::VectorXd& coeffs, const Parameters& params) : coeffs_(coeffs), p_(params) { }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    fg[0] = 0;

    // Reference State Cost
    for (int t = 0; t < N; ++t) {
      fg[0] += p_.w_cte*CppAD::pow(vars[cte_start + t], 2);
      fg[0] += p_.w_epsi*CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += p_.w_delta_v*CppAD::pow(vars[v_start + t]-p_.v_target, 2);
    }

    for (int t = 0; t < N-1; ++t) {
      fg[0] += p_.w_steer*CppAD::pow(vars[delta_start +t], 2);
      fg[0] += p_.w_acc*CppAD::pow(vars[a_start +t], 2);
    }
    for (int t = 0; t < N-2; ++t) {
      fg[0] += p_.w_steer_delta*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += p_.w_acc_delta*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
    //
    // Setup Constraints

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start]    = vars[x_start];
    fg[1 + y_start]    = vars[y_start];
    fg[1 + psi_start]  = vars[psi_start];
    fg[1 + v_start]    = vars[v_start];
    fg[1 + cte_start]  = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      AD<double> x1    = vars[x_start    + t];
      AD<double> y1    = vars[y_start    + t];
      AD<double> psi1  = vars[psi_start  + t];
      AD<double> v1    = vars[v_start    + t];
      AD<double> cte1  = vars[cte_start  + t];
      AD<double> epsi1 = vars[epsi_start + t];

      AD<double> x0    = vars[x_start    + t - 1];
      AD<double> y0    = vars[y_start    + t - 1];
      AD<double> psi0  = vars[psi_start  + t - 1];
      AD<double> v0    = vars[v_start    + t - 1];
      AD<double> cte0  = vars[cte_start  + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0     = vars[a_start     + t - 1];

      AD<double> f0      = polyeval(coeffs_, x0);
      AD<double> psides0 = CppAD::atan(polydereval(coeffs_, x0));

      fg[1 + x_start    + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start    + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start  + t] = psi1 - (psi0 + v0 * delta0 * dt / Lf);
      fg[1 + v_start    + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start  + t] = cte1 - ((f0-y0) + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 * dt / Lf);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC(const Parameters& params) : p_(params) {
  N = p_.N;
  dt = p_.dt;
  x_start     = 0;
  y_start     = x_start + N;
  psi_start   = y_start + N;
  v_start     = psi_start + N;
  cte_start   = v_start + N;
  epsi_start  = cte_start + N;
  delta_start = epsi_start + N;
  a_start     = delta_start + N - 1;
}
MPC::~MPC() {}

std::vector<double> MPC::Solve(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = 6*N+2*(N-1);
  size_t n_constraints = 6*N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  vars[x_start]    = state[0];
  vars[y_start]    = state[1];
  vars[psi_start]  = state[2];
  vars[v_start]    = state[3];
  vars[cte_start]  = state[4];
  vars[epsi_start] = state[5];

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set lower and upper limits for variables.
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start]    = state[0];
  constraints_lowerbound[y_start]    = state[1];
  constraints_lowerbound[psi_start]  = state[2];
  constraints_lowerbound[v_start]    = state[3];
  constraints_lowerbound[cte_start]  = state[4];
  constraints_lowerbound[epsi_start] = state[5];

  constraints_upperbound[x_start]    = state[0];
  constraints_upperbound[y_start]    = state[1];
  constraints_upperbound[psi_start]  = state[2];
  constraints_upperbound[v_start]    = state[3];
  constraints_upperbound[cte_start]  = state[4];
  constraints_upperbound[epsi_start] = state[5];

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, p_);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.1\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  std::vector<double> result(2*N + 2);
  for (int i = 0; i < psi_start; ++i) {
    result[i] = solution.x[i];
  }
  result[2*N]   = solution.x[delta_start + p_.control_idx];
  result[2*N+1] = solution.x[a_start     + p_.control_idx];
  return result;
}
