#pragma once
#include "Rocket.h"
#include <vector>
#include "nlopt.hpp"


class Controller
{
public:

	Input computeControl(); //need current state, target state
private:
	const Rocket& m_rocket;   
	RocketState m_targetState;
	int m_horizon;
	// some Matrix m_R;
	// we really should have a model here
	nlopt::opt optimizer;

	void setupOptimizer();
	bool optimize(std::vector<Input>& u);
	static double objectiveFunctionWrapper(const std::vector<double>& u, std::vector<double>& grad, void* data);
	// predictStates? can use m_rocket to do that

	double computeCost(const std::vector<RocketState>& predictedStates, const std::vector<Input>& inputs);
};
