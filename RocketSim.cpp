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
    // Rocket Constants
    double rocketLength{20.0};
    double rocketRadius{ 3.0 };
    double rocketMass{ 100000.0 };

    // Mission Constants
    const double burnDuration = 15.0; 
    const double dt = 0.1;
    Vector3D targetPos{ 0.0, 0.0, 10.0 };   
    RocketState targetState{targetPos};
    int horizon{ 3 }; // horizon for mpc control
    
    Rocket MaxiRocket{ rocketLength, rocketRadius, rocketMass };
    Controller mpcController(MaxiRocket, targetState, dt, horizon);

    for (double time = 0; time < burnDuration; time += dt) {
        Input input = mpcController.computeControl(MaxiRocket.getState(), targetState);  // returns optimal control for current time step
        MaxiRocket.update(input, dt);                                                  // updates the state of rocket given the input
    }

    std::cout << "Rocket burn complete.\n";
    MaxiRocket.printState();
    MaxiRocket.plotTrajectory();

    return 0;
}
