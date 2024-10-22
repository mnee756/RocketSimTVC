// RocketSim.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <cassert>
#include "Vector3D.h"
#include "Rocket.h"
#include "matplotlibcpp.h"
#include "Controller.h"

namespace plt = matplotlibcpp;
int main() {
    double rocketLength{20.0};
    double rocketRadius{ 3.0 };
    double rocketMass{ 100000.0 };
    Rocket MaxiRocket{ rocketLength, rocketRadius, rocketMass };

    const double burnDuration = 15.0; 
    const double dt = 0.1;
    Vector3D targetPos{ 0.0, 0.0, 10.0 };
    RocketState targetState{targetPos};

    int horizon{ 2 };
    Controller mpcController(MaxiRocket, targetState, dt, horizon);

    for (double time = 0; time < burnDuration; time += dt) {
        Input input = mpcController.computeControl(MaxiRocket.getState());
        // std::cout << input;
        MaxiRocket.dynamics(input, dt); // Call dynamics method to update rocket state
    }

    std::cout << "Rocket burn complete.\n";
    MaxiRocket.printState();
    
    MaxiRocket.plotTrajectory();

    return 0;
}
