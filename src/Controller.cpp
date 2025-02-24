#include "Controller.h"
#include <vector>

Input Controller::computeControl(RocketState currentState, RocketState targetState)
{
    m_currentState = currentState;
    m_targetState = targetState;
    std::vector<Input> optimalControls(m_horizon);

    // Call optimize to get the optimal control inputs
    if (optimize(optimalControls)) 
    {
        return optimalControls[0]; // take the first input of the input sequence
    }
    else {
        std::cerr << "Optimization failed!" << std::endl;
        return Input(); 
    }
}

void Controller::setupOptimizer()
{
    
    int numVars = m_horizon * 12; // 8 gimbal angles + 4 throttle  
    optimizer = nlopt::opt(nlopt::LD_SLSQP, numVars);
    
    
    std::vector<double> lowerBounds;
    std::vector<double> upperBounds;

    for (int i = 0; i < m_horizon; ++i) {
        // Bounds for gimbal angles
        lowerBounds.insert(lowerBounds.end(), 8, deg2rad(-20.0));
        upperBounds.insert(upperBounds.end(), 8, deg2rad(20.0));

        // Bounds for throttle 
        lowerBounds.insert(lowerBounds.end(), 4, 0.0); 
        upperBounds.insert(upperBounds.end(), 4, 1.0); 
    }

    optimizer.set_lower_bounds(lowerBounds);
    optimizer.set_upper_bounds(upperBounds);
    optimizer.set_min_objective(objectiveFunctionWrapper, this);
    
    optimizer.set_ftol_rel(1e-5);
    optimizer.set_maxeval(8000);

    // Optional: Set step sizes
    //std::vector<double> stepSizes(numVars, 0.01);
    //optimizer.set_initial_step(stepSizes);
    std::cout << "Optimizer setup complete." << std::endl;

}

bool Controller::optimize(std::vector<Input>& u)
{
    // Flatten the entire vector of Input for the optimizer
    std::vector<double> initialGuess = flattenInput(u);  // might not be a bad idea to initialize the initial guess as the previous control...
    double minf;
    
    // Run the optimization process
    nlopt::result result = optimizer.optimize(initialGuess, minf);

    if (result < 0) {
        std::cerr << "Optimization failed!" << std::endl;
        return false; 
    }

    // Convert back to Input
    u = unflattenInput(initialGuess);

    return true;
}

double Controller::objectiveFunction(const std::vector<double>& u, std::vector<double>& grad, void* data) {

    std::vector<RocketState> predictedStates;
    std::vector<Input> inputs = unflattenInput(u); // Vector to store inputs for computeCost

    predictedStates = predictStates(inputs);    
    double cost = computeCost(predictedStates, inputs); 

    // Gradient Calculation 
    if (!grad.empty()) {
        const double epsilon = 1e-3;
        std::vector<double> u_perturbed = u;
        
        // Loop over each control variable to approximate the gradient
        for (int i = 0; i < u.size(); ++i) {

            // Perturb the control variable
            u_perturbed[i] += epsilon;
            std::vector<Input> perturbed_inputs = unflattenInput(u_perturbed); 
            //inneficient. maybe don't bother with u_perturbed, just loop through inputs instead... But at readability cost.
            
            std::vector<RocketState> perturbedPredictedStates = predictStates(perturbed_inputs);
            double perturbedCost = computeCost(perturbedPredictedStates, perturbed_inputs);

            // Compute the gradient as the change in cost over the perturbation
            grad[i] = (perturbedCost - cost) / epsilon;
            
            u_perturbed[i] = u[i]; // Reset the perturbed input
        }
    }
    return cost; 
}

double Controller::objectiveFunctionWrapper(const std::vector<double>& u, std::vector<double>& grad, void* data)
{
    // Cast the void pointer back to Controller
    Controller* controller = static_cast<Controller*>(data);

    // Call the member function and return its result
    return controller->objectiveFunction(u, grad, data);
}

std::vector<RocketState> Controller::predictStates(const std::vector<Input>& inputs)
{
    // Predict the next states given the current state of the rocket and a sequence of inputs
    std::vector<RocketState> predictedStates;
    RocketState currentState = m_currentState;

    for (const auto& input : inputs)
    {
        currentState = m_rocket.dynamics(currentState, input, m_dt);
        predictedStates.push_back(currentState);
    }

    return predictedStates;
}


double Controller::computeCost(std::vector<RocketState>& predictedStates, std::vector<Input>& inputs)
{
    double totalCost = 0.0;    

    for (int i = 0; i < predictedStates.size(); ++i)
    {
        resetError();

        // Tracking error cost
        trackingErr = computeTrackingError(predictedStates[i], m_targetState);
        controlEffort = computeControlEffort(inputs[i]);
        tiltErr = computeTiltError(predictedStates[i]);
        velErr = computeVelocityError(predictedStates[i]);


        totalCost += trackingWeight * trackingErr;
        //std::cout << "tracking cost: " << totalCost << '\n';
        totalCost += controlEffortWeight * controlEffort;
        //std::cout << "control effort cost: " << controlEffortWeight * controlEffort << '\n';
        totalCost += tiltWeight * tiltErr;
        //std::cout << "tilt cost: " << tiltWeight * tiltCost << '\n';
        totalCost += velWeight * velErr;
    }

    return totalCost;
}

std::vector<double> Controller::flattenInput(const std::vector<Input>& inputs) {
    std::vector<double> flat;

    for (const auto& input : inputs) {
        for (const auto& angles : input.gimbalAngles) {
            flat.insert(flat.end(), angles.begin(), angles.end());
        }
        flat.insert(flat.end(), input.throttle.begin(), input.throttle.end());
    }
    return flat;
}

std::vector<Input> Controller::unflattenInput(const std::vector<double>& flat) {
    std::vector<Input> inputs;
    int index = 0;

    // Loop for the number of time steps (m_horizon)
    for (int t = 0; t < m_horizon; ++t) {
        Input input;

        // Assuming gimbalAngles is 4x2
        for (int i = 0; i < 4; ++i) {
            input.gimbalAngles[i][0] = flat[index++];
            input.gimbalAngles[i][1] = flat[index++];
        }

        // Unflatten throttle
        for (int i = 0; i < 4; ++i) {
            input.throttle[i] = flat[index++];
        }

        inputs.push_back(input);
    }

    return inputs;
}
double Controller::computeTrackingError(RocketState& state, RocketState& target)
{ 
    return (state.pos - m_targetState.pos).magnitude(); 
}

double Controller::computeControlEffort(Input& input)
{
    double CE = 0;
    for (int i = 0; i < input.throttle.size(); ++i)
    {
        CE += std::pow(input.throttle[i], 2);
        CE += std::pow(input.gimbalAngles[i][0], 2) +
            std::pow(input.gimbalAngles[i][1], 2);
    }
    return CE;
}

double Controller::computeTiltError(RocketState& state)
{
    // Deviation from vertical Cost
    Vector3D zAxis = m_rocket.transformToWorldFrame(Vector3D(0, 0, 1), state); // Use your transform function here
    double tiltDeviation = 1.0 - std::abs(zAxis.dot(Vector3D(0, 0, 1))); // Deviation from vertical (dot product with world Z-axis)
    double tiltCost = tiltDeviation * tiltDeviation;
    return tiltCost;
}

double Controller::computeVelocityError(RocketState& state)
{
    return state.vel.magnitude();
    // TODO right now target is 0 vel...
}