// RocketSim.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <cassert>
#include "Vector3D.h"
#include "Point3D.h"
#include "Rocket.h"

int main() {

    std::cout << "Hello World!\n";

    double rocketLength{ 20.0 };
    double rocketRadius{ 3.0 };
    double rocketMass{ 100000.0 };
    Rocket MaxiRocket{ rocketLength, rocketRadius, rocketMass };

    const double burnDuration = 10.0; 
    const double dt = 0.1;
    //MaxiRocket.
    for (double time = 0; time < burnDuration; time += dt) {
        MaxiRocket.dynamics(dt); // Call dynamics method to update position/velocity
    }

    MaxiRocket.printState();
    std::cout << "Rocket burn complete.\n";

    return 0;
}
