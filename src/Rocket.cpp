
#include "Rocket.h"
#include "constants.h"
#include "Matrix.h"
#include "Vector3D.h"

void Rocket::initEngines(double rad) 
{
    double engineLength{ 4.0 };
    double engineRadialPosition{ rad / 2.0 };
    double thrust = 250000;

    m_engines.emplace_back(Vector3D{  engineRadialPosition,  0, engineLength }, engineLength, thrust, 1); // right
    m_engines.emplace_back(Vector3D{  0,  engineRadialPosition, engineLength }, engineLength, thrust, 1); // top
    m_engines.emplace_back(Vector3D{ -engineRadialPosition,  0, engineLength }, engineLength, thrust, 1); // left
    m_engines.emplace_back(Vector3D{  0, -engineRadialPosition, engineLength }, engineLength, thrust, 1); // bottom
}

void Rocket::dynamics(double dt)
{
    Vector3D totalForce{};
    Vector3D totalMoment{};
    
    m_engines[0].setGimbalAngles({ 1.0 / 180.0 * M_PI, 0.0, 0.0 });
    m_engines[2].setGimbalAngles({ 1.0 / 180.0 * M_PI, 0.0, 0.0 });

    m_engines[1].setGimbalAngles({ 0.0, 1.0 / 180.0 * M_PI, 0.0 });
    m_engines[3].setGimbalAngles({ 0.0, 1.0 / 180.0 * M_PI, 0.0 });

    // get the forces and torques on rocket
    totalForce += Vector3D{0.0, 0.0, -m_mass * GRAVITY}; // gravity
    for (auto& engine : m_engines)
    {
        Vector3D thrust = engine.getRengine2rocket() * engine.getThrust();
        totalForce += thrust;
        Vector3D r = engine.getGimbalPoint() - m_cg;
        totalMoment += r.cross(thrust);
    }

    m_state.accel = totalForce / m_mass;
    m_state.vel += m_state.accel * dt;
    m_state.pos += m_state.vel * dt;

    m_state.angAccel = m_inertia.inverse() * totalMoment;
    m_state.angVel += m_state.angAccel * dt;
    m_state.ang += m_state.angVel * dt;

    m_data.push_back(m_state); 
}