#include "Rocket.h"

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