// RocketSim.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <cassert>
#include "Vector3D.h"
#include "Rocket.h"
#include "matplotlibcpp.h"
#include "Controller.h"
#include <array>

namespace plt = matplotlibcpp;
int main() {

    // Rocket Constants
    constexpr double rocketLength{20.0};
    constexpr double rocketRadius{ 3.0 };
    constexpr double rocketMass{ 90000.0 };

    // Simulation Constants
    const Vector3D initialPos{ -5, 5, 30 };
    constexpr double burnDuration = 20.0; 
    constexpr double s_dt = 0.01;
    constexpr int numSteps = static_cast<int>(burnDuration / s_dt);
    std::array<double, numSteps> timeArray;
    
    // Controller Contants
    constexpr int c_horizon{ 13 };           // horizon steps for MPC. c_horizon * c_dt is the time horizon
    constexpr double c_dt = 0.1;            // time steps used for controller
    constexpr double c_tUpdate = 0.1;       // time between MPC predictions
    constexpr double trackingWeight = 2.0;
    constexpr double controlEffortWeight = 0.0;       
    constexpr double tiltWeight = 5e2;
    constexpr double velWeight = 1.2;
    
    //Vector3D targetPos{ 1.0, 0.5, 10.0 };   
    Vector3D targetPos{ 0, 0, 0.0 };
    RocketState targetState{targetPos};
    Vector3D targetPos2{ 1.0, 1.0, 12.0 };
    RocketState targetState2{ targetPos2 };
    
    // Object Creation
    Rocket MaxiRocket{ rocketLength, rocketRadius, rocketMass, initialPos};
    Controller mpcController(MaxiRocket, targetState, c_dt, c_horizon, 
        trackingWeight, controlEffortWeight, tiltWeight, velWeight);


    // Main Simulation Loop
    double lastUpdateTime = -99.0; // initialized negative so the controller activates on the first time step
    Input input;
    for (double i = 0; i < numSteps; i++) {
        double time = i * s_dt;
        timeArray[i] = i * s_dt;
        /*if (time > 14)
        {    
            targetState = targetState2;
        }*/

        if (time - lastUpdateTime >= c_tUpdate) {
            input = mpcController.computeControl(MaxiRocket.getState(), targetState);
            lastUpdateTime = time;
        }

        // Updates the state of rocket given the input
        MaxiRocket.update(input, s_dt); 
    }

    std::cout << "Rocket burn complete.\n";
    MaxiRocket.printState();
    MaxiRocket.plotTrajectory(s_dt);

    return 0;
}
