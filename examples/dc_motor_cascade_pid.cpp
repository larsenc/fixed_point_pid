#define WITHOUT_NUMPY

#include <fixed_point/scaled_int.hpp>
#include <fixed_point_pid/pid.hpp>

#ifdef _DEBUG
	#undef _DEBUG
		#include <python.h>
	#define _DEBUG
#else
	#include <python.h>
#endif

#include <Eigen/Dense>

#include <matplotlibcpp.h>

#include <vector>

namespace plt = matplotlibcpp;

struct StateSpaceModel
{
	const Eigen::MatrixXd A;
	const Eigen::VectorXd B;
	const Eigen::MatrixXd C;
	const Eigen::VectorXd D;
};

StateSpaceModel dcMotor(double Ra, double La, double Kv, double Ka, double f, double J)
{
	Eigen::MatrixXd A(2, 2);
	A <<	-Ra / La, -Kv / La,
			Ka / J, -f / J;

	Eigen::VectorXd B(2);
	B <<	1.0 / La, 0.0;

	Eigen::MatrixXd C(2, 2);
	C <<	1.0, 0.0,
			0.0, 1.0;
	Eigen::VectorXd D(2);
	D <<	0.0, 0.0;

	return { A, B, C, D };
}

/**
* Take a step with using the Euler forward method
* x[k+1] = (I + A*dt)*x[k] + dt*B*u[k]
*/
Eigen::VectorXd step(double dt, const StateSpaceModel& model, Eigen::VectorXd& states, double u)
{
	const auto N = states.size();
	const auto I = Eigen::MatrixXd::Identity(N, N);

	states = (I + model.A * dt) * states + model.B * u * dt;
	return model.C * states + model.D * u;
}

int main()
{
	using namespace fixed_point;
	using namespace fixed_point::control;

	// Motor constants
	const double Ra = 8.4;
	const double La = 1.13e-3;
	const double Kv = 0.042;
	const double Ka = 0.042;
	const double f = 0.001;
	const double J = 4.0e-6;

	// Simulation parameters
	double position = 0.0;
	double voltage = 0.0;
	double t = 0.0;
	const double tEnd = 0.02;
	const double dt = 0.0001; // 10 kHz

	// Current regulator
	typedef scaled_int<1, 14>	CurrentInput_t;
	typedef scaled_int<5, 10>	VoltageOutput_t;

	typedef scaled_int<7, 8>	CurrentKp_t;
	typedef scaled_int<2, 13>	CurrentKi_t;

	VoltageOutput_t minVoltageOutput(unscaled_double<5, 10>(-12.0));
	VoltageOutput_t maxVoltageOutput(unscaled_double<5, 10>(12.0));

	CurrentKp_t ckp(unscaled_double<7, 8>(2.0));
	CurrentKi_t cki(unscaled_double<2, 13>(1.0));
	PIController<CurrentInput_t, VoltageOutput_t, CurrentKp_t, CurrentKi_t> currentPI(ckp, cki, minVoltageOutput, maxVoltageOutput);

	// Speed regulator
	typedef scaled_int<6, 9>	VelocityInput_t;
	typedef CurrentInput_t		CurrentOutput_t;

	typedef scaled_int<7, 8>	VelocityKp_t;
	typedef scaled_int<2, 13>	VelocityKi_t;

	CurrentOutput_t minCurrentOutput(unscaled_double<1, 14>(-0.8));
	CurrentOutput_t maxCurrentOutput(unscaled_double<1, 14>(0.8));

	VelocityKp_t vkp(unscaled_double<7, 8>(0.075));
	VelocityKi_t vki(unscaled_double<2, 13>(0.002));
	PIController<VelocityInput_t, CurrentOutput_t, VelocityKp_t, VelocityKi_t> velocityPI(vkp, vki, minCurrentOutput, maxCurrentOutput);

	const double initialCurrent = 0.0;
	const double initialVelocity = 0.0;

	Eigen::VectorXd states(2, 1);
	states << initialCurrent, initialVelocity;

	StateSpaceModel model = dcMotor(Ra, La, Kv, Ka, f, J);

	std::vector<double> ts;
	std::vector<double> currents;
	std::vector<double> velocities;
	std::vector<double> positions;
	std::vector<double> voltages;

	VelocityInput_t refVelocity(unscaled_double<6, 9>(1.0));

	while (t <= tEnd) {
		states = step(dt, model, states, voltage);

		ts.push_back(t);
		currents.push_back(states[0]);
		position += states[1] * dt;
		positions.push_back(position);
		velocities.push_back(states[1]);
		voltages.push_back(voltage);

		// Calculate inner current PI controller output
		VelocityInput_t measuredVelocity(unscaled_double<6, 9>((double)states[1]));
		CurrentOutput_t referenceCurrent = velocityPI.calculateOutput(refVelocity, measuredVelocity);

		// Calculate outer velocity PI controller output
		CurrentInput_t measuredCurrent(unscaled_double<1, 14>((double)states[0]));
		voltage = currentPI.calculateOutput(referenceCurrent, measuredCurrent).unscale<float>();

		t += dt;
	}

	plt::subplot(4, 1, 1);
	plt::title("Current [A]");
	plt::plot(ts, currents);

	plt::subplot(4, 1, 2);
	plt::title("Position [rad]");
	plt::plot(ts, positions);

	plt::subplot(4, 1, 3);
	plt::title("Velocity [rad/s]");
	plt::plot(ts, velocities);
	plt::plot(
		std::vector<double>{0.0, tEnd},
		std::vector<double>{refVelocity.unscale<double>(), refVelocity.unscale<double>()}
	);

	plt::subplot(4, 1, 4);
	plt::title("Voltage [V]");
	plt::plot(ts, voltages);

	plt::show();

	return 0;
}