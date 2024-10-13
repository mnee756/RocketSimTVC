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


    return 0;
}
