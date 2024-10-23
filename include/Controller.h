#pragma once
#include "Rocket.h"
#include <vector>
#include "nlopt.hpp"


class Controller
{
public:
	Controller(Rocket rocket, RocketState targetState, double dt, int horizon)
		: m_rocket(rocket), m_currentState(rocket.getState()), m_targetState(targetState), m_dt(dt), m_horizon(horizon)
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

	void setupOptimizer();
	bool optimize(std::vector<Input>& u);
	double objectiveFunction(const std::vector<double>& u, std::vector<double>& grad, void* data);
	static double objectiveFunctionWrapper(const std::vector<double>& u, std::vector<double>& grad, void* data);
	std::vector<RocketState> predictStates(const std::vector<Input>& inputs);
	double computeCost(std::vector<RocketState>& predictedStates, std::vector<Input>& inputs);

	std::vector<double> flattenInput(const std::vector<Input>& input);
	std::vector<Input> unflattenInput(const std::vector<double>& flat);
};
