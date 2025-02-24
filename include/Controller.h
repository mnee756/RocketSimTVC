#pragma once
#include "Rocket.h"
#include <vector>
#include "nlopt.hpp"


class Controller
{
public:
	Controller(Rocket rocket, RocketState targetState, double dt, int horizon, double W_tracking = 1, double W_controlEffort = 0, double W_tilt = 1e3,
		double W_vel = 0)
		: m_rocket(rocket), m_currentState(rocket.getState()), m_targetState(targetState), m_dt(dt), m_horizon(horizon),
		trackingWeight(W_tracking), controlEffortWeight(W_controlEffort), tiltWeight(W_tilt), velWeight(W_vel),
		trackingErr(0.0), controlEffort(0.0), tiltErr(0.0), velErr(0.0)
	{
		setupOptimizer();
	}
	Input computeControl(RocketState currentState, RocketState targetState); 
private:
	Rocket m_rocket;   
	RocketState m_currentState;
	RocketState m_targetState;
	double m_dt;
	int m_horizon;
	// some Matrix m_R (see computeCost)
	nlopt::opt optimizer;

	double trackingWeight;
	double trackingErr;
	double controlEffortWeight;
	double controlEffort;
	double tiltWeight;
	double tiltErr;
	double velWeight;
	double velErr;
	void resetError() { trackingErr = 0; controlEffort = 0; tiltErr = 0; };
	double computeTrackingError(RocketState& state, RocketState& target);
	double computeControlEffort(Input& input);
	double computeTiltError(RocketState& state);
	double computeVelocityError(RocketState& state);


	void setupOptimizer();
	bool optimize(std::vector<Input>& u);
	double objectiveFunction(const std::vector<double>& u, std::vector<double>& grad, void* data);
	static double objectiveFunctionWrapper(const std::vector<double>& u, std::vector<double>& grad, void* data);
	std::vector<RocketState> predictStates(const std::vector<Input>& inputs);
	double computeCost(std::vector<RocketState>& predictedStates, std::vector<Input>& inputs);

	std::vector<double> flattenInput(const std::vector<Input>& input);
	std::vector<Input> unflattenInput(const std::vector<double>& flat);
};
