#include "Controller.h"

/*Input Controller::computeControl()
{
    std::vector<Input> optimalControls(m_horizon, std::vector<double>(m_numInputs));
    m_currentTemps = currentTemps;

    // Call optimize to get the optimal control inputs
    if (optimize(optimalControls)) {
        // Use the first control input as the output (can return all if needed)
        return optimalControls[0]; // Adjust as needed to return appropriate control
    }
    else {
        // Handle the failure case (could log or throw an error)
        std::cerr << "Optimization failed!" << std::endl;
        return std::vector<double>(m_numInputs, 0.0); // Return a default vector or handle as needed
    }
}*/

void Controller::setupOptimizer()
{

}

bool Controller::optimize(std::vector<Input>& u)
{

}

double Controller::objectiveFunctionWrapper(const std::vector<double>& u, std::vector<double>& grad, void* data)
{

}


double Controller::computeCost(const std::vector<RocketState>& predictedStates, const std::vector<Input>& inputs)
{

}