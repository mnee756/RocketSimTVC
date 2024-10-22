#include "Controller.h"
#include <vector>

std::vector<double> flattenInput(const Input& input);
Input unflattenInput(const std::vector<double>& flat);


Input Controller::computeControl(RocketState currentState)
{
    m_currentState = currentState;
    std::vector<Input> optimalControls(m_horizon);

    // Call optimize to get the optimal control inputs
    if (optimize(optimalControls)) 
    {
        return optimalControls[0]; // take the first input of the input sequence
    }
    else {
        std::cerr << "Optimization failed!" << std::endl;
        return Input(); // Return a default constructed Input struct
    }
}

void Controller::setupOptimizer()
{
    
    int numVars = m_horizon * 12; // 8 gimbal angles + 4 throttle values
    optimizer = nlopt::opt(nlopt::LD_SLSQP, numVars);
    
    // Set specific lower and upper bounds for throttle and gimbal angles
    std::vector<double> lowerBounds;
    std::vector<double> upperBounds;

    for (int i = 0; i < m_horizon; ++i) {
        // Bounds for gimbal angles (8 values, assuming range is -30 to 30 degrees)
        lowerBounds.insert(lowerBounds.end(), 8, deg2rad(-20.0));
        upperBounds.insert(upperBounds.end(), 8, deg2rad(20.0));

        // Bounds for throttle (4 values)
        lowerBounds.insert(lowerBounds.end(), 4, 0.0); 
        upperBounds.insert(upperBounds.end(), 4, 1.0); 
    }

    optimizer.set_lower_bounds(lowerBounds);
    optimizer.set_upper_bounds(upperBounds);
    optimizer.set_min_objective(objectiveFunctionWrapper, this);
    
    optimizer.set_ftol_rel(1e-5);
    //optimizer.set_maxeval(1000);

    // Optional: Set step sizes
    //std::vector<double> stepSizes(numVars, 0.01);
    //optimizer.set_initial_step(stepSizes);
    std::cout << "Optimizer setup complete." << std::endl;

}

bool Controller::optimize(std::vector<Input>& u)
{
    // Flatten the entire vector of Input for the optimizer
    std::vector<double> initialGuess;
    for (const auto& input : u) {
        auto flatInput = flattenInput(input);
        initialGuess.insert(initialGuess.end(), flatInput.begin(), flatInput.end());
    }
    double minf;
    
    // Run the optimization process
    nlopt::result result = optimizer.optimize(initialGuess, minf);

    if (result < 0) {
        std::cerr << "Optimization failed!" << std::endl;
        return false; // Handle optimization failure
    }

    // Convert back to Input
    for (int i = 0; i < u.size(); ++i) {
        std::vector<double> subVector(initialGuess.begin() + i * 12, initialGuess.begin() + (i + 1) * 12);
        u[i] = unflattenInput(subVector); 
    }

    return true;
}

double Controller::objectiveFunction(const std::vector<double>& u, std::vector<double>& grad, void* data) {

    std::vector<RocketState> predictedStates;
    std::vector<Input> inputs; // Vector to store inputs for computeCost
    RocketState currentState = m_currentState; // Use current state

    // Reassemble vector of inputs
    for (int i = 0; i < m_horizon; ++i) {
        std::vector<double> inputSlice(u.begin() + i * 12, u.begin() + (i + 1) * 12);
        Input input = unflattenInput(inputSlice);
        inputs.push_back(input); 
    }

    predictedStates = predictStates(inputs);    
    double cost = computeCost(predictedStates, inputs); 

    // Loop to calculate the gradient if needed;
    if (!grad.empty()) {
        const double epsilon = 1e-3;
        std::vector<double> u_perturbed = u;
        
        // Loop over each control variable to approximate the gradient
        for (int i = 0; i < u.size(); ++i) {
            // Perturb the control variable
            u_perturbed[i] += epsilon;

            std::vector<Input> perturbed_inputs;
            for (int j = 0; j < m_horizon; ++j) {
                std::vector<double> inputSlice(u_perturbed.begin() + j * 12, u_perturbed.begin() + (j + 1) * 12);
                Input input = unflattenInput(inputSlice);
                perturbed_inputs.push_back(input);
            }
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
        currentState = m_rocket.update(currentState, input, m_dt);
        predictedStates.push_back(currentState);
    }

    return predictedStates;
}


double Controller::computeCost(std::vector<RocketState>& predictedStates, std::vector<Input>& inputs)
{
    double totalCost = 0.0;

    double trackingWeight = 1.0;
    double controlEffortWeight = 0.01;
    
    for (int i = 0; i < predictedStates.size(); ++i)
    {
        // Tracking error cost
        double trackingError = (predictedStates[i].pos - m_targetState.pos).magnitude();
        //
        //    + (predictedStates[i].vel - m_targetState.vel).norm() +
        //    (predictedStates[i].ang - m_targetState.ang).norm() +
        //    (predictedStates[i].angVel - m_targetState.angVel).norm();

        // Control effort costs
        double controlEffort = 0.0;
        for (int j = 0; j < inputs[i].throttle.size(); ++j)
        {
            controlEffort += std::pow(inputs[i].throttle[j], 2); 
            controlEffort += std::pow(inputs[i].gimbalAngles[j][0], 2) +
                std::pow(inputs[i].gimbalAngles[j][1], 2); 
        }

        totalCost += trackingWeight * trackingError;
        totalCost += controlEffortWeight * controlEffort;
    }

    return totalCost;
}

std::vector<double> flattenInput(const Input& input) {
    std::vector<double> flat;

    // Flatten gimbal angles (2D vector)
    for (const auto& angles : input.gimbalAngles) {
        flat.insert(flat.end(), angles.begin(), angles.end());
    }

    // Flatten throttle
    flat.insert(flat.end(), input.throttle.begin(), input.throttle.end());

    return flat;
}

Input unflattenInput(const std::vector<double>& flat) {
    Input input;

    // Assuming gimbalAngles is 4x2
    int index = 0;
    for (int i = 0; i < 4; ++i) {
        input.gimbalAngles[i][0] = flat[index++];
        input.gimbalAngles[i][1] = flat[index++];
    }

    // Unflatten throttle
    for (int i = 0; i < 4; ++i) {
        input.throttle[i] = flat[index++];
    }

    return input;
}

