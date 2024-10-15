#include "Rocket.h"
#include "constants.h"

void Rocket::initEngines(double rad) 
{
    double engineLength{ 4.0 };
    double engineRadialPosition{ rad / 2.0 };
    for (int i = -1; i <= 1; i+=2) 
    {
        for (int j = -1; j <= 1; j += 2)
        {
            Vector3D enginePos{
                i * engineRadialPosition,  // X position
                j * engineRadialPosition,  // Y position
                engineLength               // Z position (engine exit will meet rocket base)
            };
            m_engines.emplace_back(enginePos, engineLength); // Pass position to Engine constructor
        }    
    }
}

void Rocket::dynamics(double dt)
{
    Vector3D totalForce{};
    Vector3D totalMoment{};

    // get the forces and torques on rocket
    totalForce += Vector3D{0.0, 0.0, -m_mass * GRAVITY}; // gravity
    for (auto& engine : m_engines)
    {
        Vector3D thrust = engine.getRengine2rocket() * engine.getThrust();
        totalForce += thrust;
        Vector3D r = engine.getGimbalPoint() - m_cg;
        totalMoment += r.cross(thrust);
    }

    

}