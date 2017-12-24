#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

using namespace std;

double mpc_deg2rad(double deg) {
  return deg * M_PI / 180;
}

// TODO: Set the timestep length and duration
const size_t N  = 10;
const double dt = 0.1;

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

const double REF_VELOCITY = 40;
const double REF_CTE      = 0;
const double REF_EPSI     = 0;

const double COST_FACTOR_ERRORS = 1000;
const double COST_FACTOR_VELOCITY = 1;
const double COST_FACTOR_DELTA = 2000;
const double COST_FACTOR_A     = 400;
const double COST_FACTOR_DELTA_CONT = 600;
const double COST_FACTOR_A_CONT = 10;

// The start index for each variable in `vars` vector.
const size_t VAR_VEC_X_START_IDX     = 0;
const size_t VAR_VEC_Y_START_IDX     = VAR_VEC_X_START_IDX + N;
const size_t VAR_VEC_PSI_START_IDX   = VAR_VEC_Y_START_IDX + N;
const size_t VAR_VEC_V_START_IDX     = VAR_VEC_PSI_START_IDX + N;
const size_t VAR_VEC_CTE_START_IDX   = VAR_VEC_V_START_IDX + N;
const size_t VAR_VEC_EPSI_START_IDX  = VAR_VEC_CTE_START_IDX + N;
const size_t VAR_VEC_DELTA_START_IDX = VAR_VEC_EPSI_START_IDX + N;
const size_t VAR_VEC_A_START_IDX     = VAR_VEC_DELTA_START_IDX + N - 1;

// The index of each variable in `state` vector.
const int STATE_VEC_X_IDX     = 0;
const int STATE_VEC_Y_IDX     = 1;
const int STATE_VEC_PSI_IDX   = 2;
const int STATE_VEC_V_IDX     = 3;
const int STATE_VEC_CTE_IDX   = 4;
const int STATE_VEC_EPSI_IDX  = 5;

// The controlled parameters constraints
const double DELTA_MAX = mpc_deg2rad(25) * Lf;
const double DELTA_MIN = mpc_deg2rad(-25) * Lf;
const double A_MAX     = 1;
const double A_MIN     = -1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;

  FG_eval(Eigen::VectorXd coeffs) :
  coeffs(coeffs)
  {}

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // Implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below

    // Calculate the cost function
    fg[0] = 0;
    // Add errors to cost function in order to minimize the errors
    for (size_t time_slot_idx = 0; time_slot_idx < N; ++time_slot_idx ) {
      fg[0] += COST_FACTOR_ERRORS * CppAD::pow(vars[VAR_VEC_CTE_START_IDX + time_slot_idx] - REF_CTE, 2);
      fg[0] += COST_FACTOR_ERRORS * CppAD::pow(vars[VAR_VEC_EPSI_START_IDX + time_slot_idx] - REF_EPSI, 2);

      fg[0] += COST_FACTOR_VELOCITY * CppAD::pow(vars[VAR_VEC_V_START_IDX + time_slot_idx] - REF_VELOCITY, 2);
    }

    // Add actuators to cost in order to minimize their use.
    for (size_t time_slot_idx = 0; time_slot_idx < N - 1; ++time_slot_idx ) {
      fg[0] += COST_FACTOR_DELTA * CppAD::pow(vars[VAR_VEC_DELTA_START_IDX + time_slot_idx], 2);
      fg[0] += COST_FACTOR_A * CppAD::pow(vars[VAR_VEC_A_START_IDX + time_slot_idx], 2);
    }

    // Add actuator deltas in order to prevents jump in actuator values.
    for (size_t time_slot_idx = 0; time_slot_idx < N - 2; ++time_slot_idx ) {
      fg[0] += COST_FACTOR_DELTA_CONT * CppAD::pow(vars[VAR_VEC_DELTA_START_IDX + time_slot_idx + 1] - vars[VAR_VEC_DELTA_START_IDX + time_slot_idx], 2);
      fg[0] += COST_FACTOR_A_CONT * CppAD::pow(vars[VAR_VEC_A_START_IDX + time_slot_idx + 1] - vars[VAR_VEC_A_START_IDX + time_slot_idx], 2);
    }
    

    // Calculate constraints
    size_t constraint_start_idx = 1; // equals one because idx 0 is used for cost

    // constraints at time slot 0
    fg[constraint_start_idx + VAR_VEC_X_START_IDX]    = vars[VAR_VEC_X_START_IDX];
    fg[constraint_start_idx + VAR_VEC_Y_START_IDX]    = vars[VAR_VEC_Y_START_IDX];
    fg[constraint_start_idx + VAR_VEC_PSI_START_IDX]  = vars[VAR_VEC_PSI_START_IDX];
    fg[constraint_start_idx + VAR_VEC_V_START_IDX]    = vars[VAR_VEC_V_START_IDX];
    fg[constraint_start_idx + VAR_VEC_CTE_START_IDX]  = vars[VAR_VEC_CTE_START_IDX];
    fg[constraint_start_idx + VAR_VEC_EPSI_START_IDX] = vars[VAR_VEC_EPSI_START_IDX];

    for (size_t time_slot_idx = 1; time_slot_idx < N; ++time_slot_idx) {
      // Get future state (at t+1)
      AD<double> next_x    = vars[VAR_VEC_X_START_IDX + time_slot_idx];
      AD<double> next_y    = vars[VAR_VEC_Y_START_IDX + time_slot_idx];
      AD<double> next_psi  = vars[VAR_VEC_PSI_START_IDX + time_slot_idx];
      AD<double> next_v    = vars[VAR_VEC_V_START_IDX + time_slot_idx];
      AD<double> next_cte  = vars[VAR_VEC_CTE_START_IDX + time_slot_idx];
      AD<double> next_epsi = vars[VAR_VEC_EPSI_START_IDX + time_slot_idx];

      // Get curr state
      AD<double> curr_x    = vars[VAR_VEC_X_START_IDX + time_slot_idx - 1];
      AD<double> curr_y    = vars[VAR_VEC_Y_START_IDX + time_slot_idx - 1];
      AD<double> curr_psi  = vars[VAR_VEC_PSI_START_IDX + time_slot_idx - 1];
      AD<double> curr_v    = vars[VAR_VEC_V_START_IDX + time_slot_idx - 1];
      AD<double> curr_cte  = vars[VAR_VEC_CTE_START_IDX + time_slot_idx - 1];
      AD<double> curr_epsi = vars[VAR_VEC_EPSI_START_IDX + time_slot_idx - 1];

      // Get curr actuator values
      AD<double> curr_delta = vars[VAR_VEC_DELTA_START_IDX + time_slot_idx - 1];
      AD<double> curr_a     = vars[VAR_VEC_A_START_IDX + time_slot_idx - 1];

      AD<double> curr_x_2   = curr_x * curr_x;
      AD<double> curr_x_3   = curr_x_2 * curr_x;

      AD<double> curr_f       = coeffs[0] + coeffs[1] * curr_x + coeffs[2] * curr_x_2 + coeffs[3] * curr_x_3;
      AD<double> curr_psi_des = CppAD::atan(coeffs[1] + 2 * coeffs[2] * curr_x + 3 * coeffs[3] * curr_x_2);

      // Predict future state using the model.
      AD<double> pred_x    = curr_x + curr_v * CppAD::cos(curr_psi) * dt;
      AD<double> pred_y    = curr_y + curr_v * CppAD::sin(curr_psi) * dt;
      AD<double> pred_psi  = curr_psi - curr_v * curr_delta / Lf * dt;
      AD<double> pred_v    = curr_v + curr_a * dt;
      AD<double> pred_cte  = curr_f - curr_y + curr_v * CppAD::sin(curr_epsi) * dt;
      AD<double> pred_epsi = curr_psi - curr_psi_des - curr_v * curr_delta / Lf * dt;

      // Set the future state constrains
      fg[constraint_start_idx + VAR_VEC_X_START_IDX + time_slot_idx]    = next_x - pred_x;
      fg[constraint_start_idx + VAR_VEC_Y_START_IDX + time_slot_idx]    = next_y - pred_y;
      fg[constraint_start_idx + VAR_VEC_PSI_START_IDX + time_slot_idx]  = next_psi - pred_psi;
      fg[constraint_start_idx + VAR_VEC_V_START_IDX + time_slot_idx]    = next_v - pred_v;
      fg[constraint_start_idx + VAR_VEC_CTE_START_IDX + time_slot_idx]  = next_cte - pred_cte;
      fg[constraint_start_idx + VAR_VEC_EPSI_START_IDX + time_slot_idx] = next_epsi - pred_epsi;
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t var_idx;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double curr_x    = state[STATE_VEC_X_IDX];
  double curr_y    = state[STATE_VEC_Y_IDX];
  double curr_psi  = state[STATE_VEC_PSI_IDX];
  double curr_v    = state[STATE_VEC_V_IDX];
  double curr_cte  = state[STATE_VEC_CTE_IDX];
  double curr_epsi = state[STATE_VEC_EPSI_IDX];

  // Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars   = N * state.size() + (N - 1) * 2;

  //Set the number of constraints
  size_t n_constraints = N * state.size();

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (var_idx = 0; var_idx < n_vars; ++var_idx) {
    vars[var_idx] = 0;
  }

  #if 0

  // Set initial variable values
  vars[VAR_VEC_X_START_IDX]    = curr_x;
  vars[VAR_VEC_Y_START_IDX]    = curr_y;
  vars[VAR_VEC_PSI_START_IDX]  = curr_psi;
  vars[VAR_VEC_V_START_IDX]    = curr_v;
  vars[VAR_VEC_CTE_START_IDX]  = curr_cte;
  vars[VAR_VEC_EPSI_START_IDX] = curr_epsi;
#endif

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set lower and upper limits for state variables.
  double max_value = 1.0e19;
  double min_value = -1.0e19;
  for (var_idx = 0; var_idx < VAR_VEC_DELTA_START_IDX; ++var_idx) {
    vars_lowerbound[var_idx] = min_value;
    vars_upperbound[var_idx] = max_value;
  }

  // Set lower and upper limits for state variables.
  max_value = DELTA_MAX;
  min_value = DELTA_MIN;
  for (var_idx = VAR_VEC_DELTA_START_IDX; var_idx < VAR_VEC_A_START_IDX; ++var_idx) {
    vars_lowerbound[var_idx] = min_value;
    vars_upperbound[var_idx] = max_value;
  }

  // Set lower and upper limits for state variables.
  max_value = A_MAX;
  min_value = A_MIN;
  for (var_idx = VAR_VEC_A_START_IDX; var_idx < n_vars; ++var_idx) {
    vars_lowerbound[var_idx] = min_value;
    vars_upperbound[var_idx] = max_value;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (var_idx = 0; var_idx < n_constraints; var_idx++) {
    constraints_lowerbound[var_idx] = 0;
    constraints_upperbound[var_idx] = 0;
  }

  constraints_lowerbound[VAR_VEC_X_START_IDX]    = curr_x;
  constraints_lowerbound[VAR_VEC_Y_START_IDX]    = curr_y;
  constraints_lowerbound[VAR_VEC_PSI_START_IDX]  = curr_psi;
  constraints_lowerbound[VAR_VEC_V_START_IDX]    = curr_v;
  constraints_lowerbound[VAR_VEC_CTE_START_IDX]  = curr_cte;
  constraints_lowerbound[VAR_VEC_EPSI_START_IDX] = curr_epsi;

  constraints_upperbound[VAR_VEC_X_START_IDX]    = curr_x;
  constraints_upperbound[VAR_VEC_Y_START_IDX]    = curr_y;
  constraints_upperbound[VAR_VEC_PSI_START_IDX]  = curr_psi;
  constraints_upperbound[VAR_VEC_V_START_IDX]    = curr_v;
  constraints_upperbound[VAR_VEC_CTE_START_IDX]  = curr_cte;
  constraints_upperbound[VAR_VEC_EPSI_START_IDX] = curr_epsi;


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

  result.push_back(solution.x[VAR_VEC_DELTA_START_IDX]);
  result.push_back(solution.x[VAR_VEC_A_START_IDX]);

  for (var_idx = 0; var_idx < N - 1; ++var_idx) {
    result.push_back(solution.x[VAR_VEC_X_START_IDX + var_idx + 1]);
    result.push_back(solution.x[VAR_VEC_Y_START_IDX + var_idx + 1]);
  }

  return result;
}
