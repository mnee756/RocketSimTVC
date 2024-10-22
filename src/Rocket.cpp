
#include "Rocket.h"
#include "constants.h"
#include "Matrix.h"
#include "Vector3D.h"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

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


RocketState Rocket::update(RocketState state, Input input, double dt) 
{
    processInput(input);
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

    state.accel = totalForce / m_mass;
    state.vel += state.accel * dt;
    state.pos += state.vel * dt;

    state.angAccel = m_inertia.inverse() * totalMoment;
    state.angVel += state.angAccel * dt;
    state.ang += state.angVel * dt;
    
    return state;
}

void Rocket::dynamics(Input input, double dt)
{ 
    RocketState state = Rocket::update(m_state, input, dt);
    m_state = state;
    m_data.push_back(state); 
}

void Rocket::processInput(Input input)
{
    for (int i{ 0 }; i < m_engines.size(); i++)
    {
        m_engines[i].setGimbalAngles(input.gimbalAngles[i]);
        m_engines[i].setThrottle(input.throttle[i]);
    }
}



void Rocket::plotTrajectory() const
{
    std::vector<double> timeData(m_data.size());
    std::vector<double> xData(m_data.size());
    std::vector<double> yData(m_data.size());
    std::vector<double> zData(m_data.size());


    for (int i = 0; i < m_data.size(); ++i) {
        timeData[i] = i * 0.1;  //assumes dt =.1
        xData[i] = m_data[i].pos.getX();  
        yData[i] = m_data[i].pos.getY();  
        zData[i] = m_data[i].pos.getZ();  
    }

    plt::figure();
    plt::named_plot("X", timeData, xData, "r-");
    plt::named_plot("Y", timeData, yData, "g-");
    plt::named_plot("Z", timeData, zData, "b-");

    plt::title("Rocket Trajectory Over Time");
    plt::xlabel("Time (s)");
    plt::ylabel("Position (m)");
    plt::legend();
    plt::show();
}
