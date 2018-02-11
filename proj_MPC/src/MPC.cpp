#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

#include "util.h"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 10;
double dt = 0.1;
bool verbose = true;
int order = 3;

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
// extern double polyeval(Eigen::VectorXd coeffs, double x); 

// Evaluate a polynomial.
//extern double polyeval(Eigen::VectorXd coeffs, double x);

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
//extern Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,int order);

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    size_t x_start     = 0 + N * 0;
    size_t y_start     = 0 + N * 1;
    size_t psi_start   = 0 + N * 2;
    size_t v_start     = 0 + N * 3;
    size_t cte_start   = 0 + N * 4;
    size_t epsi_start  = 0 + N * 5;
    size_t delta_start = 0 + N * 6;
    size_t a_start     = 0 + N * 7 - 1;

    fg[0] = 0;
    AD<double> ref_v = 40.0;
    for (size_t i = 0; i < N; i ++)
    {
      fg[0] += 100 * CppAD::pow(vars[cte_start + i], 2);
      fg[0] += 100 * CppAD::pow(vars[epsi_start + i], 2);
      fg[0] += 50 * CppAD::pow(vars[v_start + i] - ref_v, 2);
    }

    // minimize the use of the actuator, and make the controller more smooth
    for (size_t i = 0; i < N - 1; i ++)
    {
      fg[0] += 50* CppAD::pow(vars[delta_start + i], 2);
      fg[0] += 50* CppAD::pow(vars[a_start + i], 2);
    }
    for (size_t i = 0; i < N - 2; i ++)
    {
      fg[0] += CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }
    // initializations, for the start time point state
    fg[x_start + 1]    = vars[x_start];
    fg[y_start + 1]    = vars[y_start];
    fg[psi_start + 1]  = vars[psi_start];
    fg[v_start + 1]    = vars[v_start];
    fg[cte_start + 1]  = vars[cte_start];
    fg[epsi_start + 1] = vars[epsi_start];

    // add constrains
    for (size_t i = 0; i < N - 1; i ++)
    {
      AD<double> x0    = vars[x_start + i];
      AD<double> y0    = vars[y_start + i];
      AD<double> psi0  = vars[psi_start + i];
      AD<double> v0    = vars[v_start + i];
      AD<double> cte0  = vars[cte_start + i];
      AD<double> epsi0 = vars[epsi_start + i];

      AD<double> delta0 = vars[delta_start + i];
      AD<double> a0     = vars[a_start + i];

      AD<double> x1    = vars[x_start + i + 1];
      AD<double> y1    = vars[y_start + i + 1];
      AD<double> v1    = vars[v_start + i + 1];
      AD<double> psi1  = vars[psi_start + i + 1];
      AD<double> cte1  = vars[cte_start + i + 1];
      AD<double> epsi1 = vars[epsi_start + i + 1];

      // the y0 refrence and psi0 reference
      // AD<double> f0 = polyeval(coeffs, x0);
      AD<double> f0;
      if (order == 1)
        f0 = coeffs[0] + coeffs[1] * x0;
      else if (order == 2)
        f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2);
      else if (order == 3)
        f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
        
      AD<double> psides0;

      if (order == 1)
        psides0 = CppAD::atan(coeffs[1]);
      else if (order == 2)
        psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 );
      else if (order == 3)
        psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2) );

      fg[x_start + i + 1]    = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[y_start + i + 1]    = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[psi_start + i + 1]  = psi1 - (psi0 - delta0 * dt * v0 / Lf);
      // remember that in the simulation, the delta will be just opposite with the car orientation, 
      // always use -delta instead of delta
      fg[v_start + i + 1]    = v1 - (v0 + a0 * dt);
      fg[cte_start + i + 1]  = cte1 - (f0 - y0  + v0 * CppAD::sin(epsi0) * dt);
      fg[epsi_start + i + 1] = epsi1 - (psi0 - psides0 - v0 * delta0 * dt / Lf);
      // use - delta to replace delta in all equations
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  if (verbose)
    cout << "MPC::SOlve() begin" << endl;

  assert(state.size() == 6);
  double x0    = state[0];
  double y0    = state[1];
  double psi0  = state[2];
  double v0    = state[3];
  double cte0  = state[4];
  double epsi0 = state[5];

  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = state.size() * N + 2 * (N - 1);
  
  // TODO: Set the number of constraints
  size_t n_constraints = N * state.size();
  // include for each state, there's must N-1 constraints

  // delta lower and upepr limit

  size_t x_start     = 0;
  size_t y_start     = x_start + N;
  size_t psi_start   = y_start + N;
  size_t v_start     = psi_start + N;
  size_t cte_start   = v_start + N;
  size_t epsi_start  = cte_start + N;
  size_t delta_start = epsi_start + N;
  size_t a_start     = delta_start + N -1;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }
  vars[x_start]    = x0;
  vars[y_start]    = y0;
  vars[psi_start]  = psi0;
  vars[v_start]    = v0;
  vars[cte_start]  = cte0;
  vars[epsi_start] = epsi0;


  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  for (size_t i = 0; i < n_vars; i ++)
  {
  	vars_lowerbound[i] = -1.0e20;
  	vars_upperbound[i] = +1.0e20;
  }
  for (size_t i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
  for (size_t i = a_start; i < n_vars; i ++)
  {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }
  // TODO: Set lower and upper limits for variables.

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  // assign initial value, so 
  constraints_lowerbound[x_start] = x0;
  constraints_upperbound[x_start] = x0;
  constraints_lowerbound[y_start] = y0;
  constraints_upperbound[y_start] = y0;
  constraints_lowerbound[psi_start] = psi0;
  constraints_upperbound[psi_start] = psi0;
  constraints_lowerbound[v_start] = v0;
  constraints_upperbound[v_start] = v0;
  constraints_lowerbound[cte_start] = cte0;
  constraints_upperbound[cte_start] = cte0;
  constraints_lowerbound[epsi_start] = epsi0;
  constraints_upperbound[epsi_start] = epsi0;


  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

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
  options += "Numeric max_cpu_time          0.5\n";

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

  vector<double> result;
  if (verbose)
  {
    cout << "solution delta = " << solution.x[delta_start] << ", solution a = " << solution.x[a_start] << endl;
  }
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);
  for (size_t i = 0; i < 10; i ++)
  {
    if (i == 0)
      cout << " the mpc future points at car coordinate are: " << endl;
    cout << "(" << solution.x[x_start + i] << ", " << solution.x[y_start + i] << ")";
  	result.push_back(solution.x[x_start + i]);
  	result.push_back(solution.x[y_start + i]);
  }
  if (verbose)
  {
    cout << endl;
    cout << "MPC::Solve() endl " << endl;
  }
  return result;

}
