#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 20;
double dt = 0.05;

double ref_cte = 0;
double ref_epsi = 0;
double ref_v = 30;

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

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;


size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
public:
	Eigen::VectorXd coeffs;
	// Coefficients of the fitted polynomial.
	FG_eval(Eigen::VectorXd coeffs) {
		this->coeffs = coeffs;
	}

	typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
	// `fg` is a vector containing the cost and constraints.
	// `vars` is a vector containing the variable values (state & actuators).
	void operator()(ADvector& fg, const ADvector& vars) {
		// The cost is stored is the first element of `fg`.
		// Any additions to the cost should be added to `fg[0]`.
		fg[0] = 0;

		// The part of the cost based on the reference state.
		for (int i = 0; i < N; i++) {
			fg[0] += CppAD::pow(vars[cte_start + i] - ref_cte, 2);
			fg[0] += 600*CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
			fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
		}

		// Minimize the use of actuators.
		for (int i = 0; i < N - 1; i++) {
			fg[0] += 5*CppAD::pow(vars[delta_start + i], 2);
			fg[0] += CppAD::pow(vars[a_start + i], 2);
		}

		// Minimize the value gap between sequential actuations.
		for (int i = 0; i < N - 2; i++) {
			fg[0] +=5*CppAD::pow(
					vars[delta_start + i + 1] - vars[delta_start + i], 2);
			fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
		}

		cout <<" fg[0]" << fg[0] << endl;

		//
		// Setup Constraints
		//
		// NOTE: In this section you'll setup the model constraints.

		// Initial constraints
		//
		// We add 1 to each of the starting indices due to cost being located at
		// index 0 of `fg`.
		// This bumps up the position of all the other values.
		fg[1 + x_start] = vars[x_start];
		fg[1 + y_start] = vars[y_start];
		fg[1 + psi_start] = vars[psi_start];
		fg[1 + v_start] = vars[v_start];
		fg[1 + cte_start] = vars[cte_start];
		fg[1 + epsi_start] = vars[epsi_start];

		// The rest of the constraints
		for (int i = 0; i < N - 1; i++) {
			// The state at time t+1 .
			AD<double> x1 = vars[x_start + i + 1];
			AD<double> y1 = vars[y_start + i + 1];
			AD<double> psi1 = vars[psi_start + i + 1];
			AD<double> v1 = vars[v_start + i + 1];
			AD<double> cte1 = vars[cte_start + i + 1];
			AD<double> epsi1 = vars[epsi_start + i + 1];

			// The state at time t.
			AD<double> x0 = vars[x_start + i];
			AD<double> y0 = vars[y_start + i];
			AD<double> psi0 = vars[psi_start + i];
			AD<double> v0 = vars[v_start + i];
			AD<double> cte0 = vars[cte_start + i];
			AD<double> epsi0 = vars[epsi_start + i];

			// Only consider the actuation at time t.

			AD<double> delta0;
			AD<double> a0 ;

			if ( i > 1)
			{

				delta0 = vars[delta_start + i -2];
				a0 = vars[a_start + i -2];
			}
			else
			{
				delta0 = vars[delta_start+i];
				a0 = vars[a_start + i];
			}


			AD<double> f0 = coeffs[0] + coeffs[1] * x0  ;
			AD<double> psides0 = CppAD::atan(coeffs[1]);

			// Here's `x` to get you started.
			// The idea here is to constraint this value to be 0.
			//
			// Recall the equations for the model:
			// x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
			// y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
			// psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
			// v_[t+1] = v[t] + a[t] * dt
			// cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
			// epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
			fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
			fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
			fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
			fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
			fg[2 + cte_start + i] = cte1
					- ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
			fg[2 + epsi_start + i] = epsi1
					- ((psi0 - psides0) + v0 * delta0 / Lf * dt);

			cout << "fg[2 + x_start + i]" << fg[2 + x_start + i] ;
		}
	}
};

//
// MPC class definition implementation.
//
MPC::MPC() {
}
MPC::~MPC() {
}

vector<double> MPC::Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs) {
	size_t i;
	typedef CPPAD_TESTVECTOR(double) Dvector;

	double x = x0[0];
	double y = x0[1];
	double psi = x0[2];
	double v = x0[3];
	double cte = x0[4];
	double epsi = x0[5];
	double speed = x0[6];
	double throttle = x0[7];

	cout << "x" << x << "psi" << psi;


	// number of independent variables
	// N timesteps == N - 1 actuations
	size_t n_vars = N * 6 + (N - 1) * 2;
	// Number of constraints
	size_t n_constraints = N * 6;

	// Initial value of the independent variables.
	// Should be 0 except for the initial values.
	Dvector vars(n_vars);
	for (int i = 0; i < n_vars; i++) {
		vars[i] = 0.0;
	}
	// Set the initial variable values
	vars[x_start] = x;
	vars[y_start] = y;
	vars[psi_start] = psi;
	vars[v_start] = v;
	vars[cte_start] = cte;
	vars[epsi_start] = epsi;
	vars[delta_start] = speed;
	vars[a_start] = throttle;

	// Lower and upper limits for x
	Dvector vars_lowerbound(n_vars);
	Dvector vars_upperbound(n_vars);

	// Set all non-actuators upper and lowerlimits
	// to the max negative and positive values.
	for (int i = 0; i < delta_start; i++) {
		vars_lowerbound[i] = -1.0e19;
		vars_upperbound[i] = 1.0e19;
	}

	// The upper and lower limits of delta are set to -25 and 25
	// degrees (values in radians).
	// NOTE: Feel free to change this to something else.
	for (int i = delta_start; i < a_start; i++) {
		vars_lowerbound[i] = -0.436332;
		vars_upperbound[i] = 0.436332;

	}

	// Acceleration/decceleration upper and lower limits.
	// NOTE: Feel free to change this to something else.
	for (int i = a_start; i < n_vars; i++) {
		vars_lowerbound[i] = -1.0;
		vars_upperbound[i] = 1.0;
	}

	// Lower and upper limits for constraints
	// All of these should be 0 except the initial
	// state indices.
	Dvector constraints_lowerbound(n_constraints);
	Dvector constraints_upperbound(n_constraints);
	for (int i = 0; i < n_constraints; i++) {
		constraints_lowerbound[i] = 0;
		constraints_upperbound[i] = 0;
	}
	constraints_lowerbound[x_start] = x;
	constraints_lowerbound[y_start] = y;
	constraints_lowerbound[psi_start] = psi;
	constraints_lowerbound[v_start] = v;
	constraints_lowerbound[cte_start] = cte;
	constraints_lowerbound[epsi_start] = epsi;


	constraints_upperbound[x_start] = x;
	constraints_upperbound[y_start] = y;
	constraints_upperbound[psi_start] = psi;
	constraints_upperbound[v_start] = v;
	constraints_upperbound[cte_start] = cte;
	constraints_upperbound[epsi_start] = epsi;


	// Object that computes objective and constraints
	FG_eval fg_eval(coeffs);

	// options
	std::string options;
	options += "Integer print_level  0\n";
	options += "Sparse  true        forward\n";
	options += "Sparse  true        reverse\n";
	options += "Numeric max_cpu_time          0.05\n";

	// place to return solution
	CppAD::ipopt::solve_result < Dvector > solution;


	// solve the problem
	CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound,
			vars_upperbound, constraints_lowerbound, constraints_upperbound,
			fg_eval, solution);

	//
	// Check some of the solution values
	//
	bool ok = true;
	ok &= solution.status == CppAD::ipopt::solve_result < Dvector > ::success;

	auto cost = solution.obj_value;
	std::cout << "Cost " << cost << std::endl;
	std::cout << "steer " << solution.x[delta_start] << std::endl;

	return {solution.x[x_start + 1],solution.x[x_start + 2] ,solution.x[x_start + 3],solution.x[x_start + 4],solution.x[x_start + 5],
		solution.x[y_start + 1],solution.x[y_start + 2],solution.x[y_start + 3],solution.x[y_start + 4],solution.x[y_start + 5],
		solution.x[psi_start + 1], solution.x[v_start + 1],
		solution.x[cte_start + 1], solution.x[epsi_start + 1],
		solution.x[delta_start], solution.x[a_start]};
}

