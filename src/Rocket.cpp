
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
    double thrust = 400000;

    m_engines.emplace_back(Vector3D{  engineRadialPosition,  0, engineLength }, engineLength, thrust, 1); // right
    m_engines.emplace_back(Vector3D{  0,  engineRadialPosition, engineLength }, engineLength, thrust, 1); // top
    m_engines.emplace_back(Vector3D{ -engineRadialPosition,  0, engineLength }, engineLength, thrust, 1); // left
    m_engines.emplace_back(Vector3D{  0, -engineRadialPosition, engineLength }, engineLength, thrust, 1); // bottom
}

Vector3D Rocket::transformToBodyFrame(Vector3D vec, RocketState& state)
{
    Quat vecq(vec.getX(), vec.getY(), vec.getZ(), 0);
    Quat Qbody = state.q.inverse() * vecq * state.q;
    return Vector3D(Qbody[0], Qbody[1], Qbody[2]);
}

Vector3D Rocket::transformToWorldFrame(Vector3D vec, RocketState& state)
{
    Quat vecq(vec.getX(), vec.getY(), vec.getZ(), 0);
    Quat Qworld = state.q * vecq * state.q.inverse();
    return Vector3D(Qworld[0], Qworld[1], Qworld[2]);
}


RocketState Rocket::dynamics(RocketState state, const Input& input, double dt) 
{
    processInput(input);
    Vector3D totalForce_b{}; // body frame forces.
    Vector3D totalMoment{};

    Vector3D g_world = Vector3D{0.0, 0.0, -m_mass * GRAVITY}; 
    totalForce_b += transformToBodyFrame(g_world, state);
    //std::cout << "gravity body: " << totalForce_b << '\n';
    for (auto& engine : m_engines)
    {
        Vector3D thrust = engine.getRengine2rocket() * engine.getThrust();
        totalForce_b += thrust;
        Vector3D r = engine.getGimbalPoint() - m_cg;
        totalMoment += r.cross(thrust);
    }

    // WORLD accel
    Vector3D accel = transformToWorldFrame(totalForce_b / m_mass, state);
    // WORLD velocity
    state.vel += accel * dt;
    // WORLD position
    state.pos += state.vel * dt;

    // BODY angaccel
    Vector3D angAccel = m_inertia.inverse() * (totalMoment - state.angVel.cross(m_inertia * state.angVel));
    // BODY angVel
    state.angVel += angAccel * dt;

    // Quaternion propagation 
    Quat q_w(state.angVel.getX(), state.angVel.getY(), state.angVel.getZ(), 0);
    state.qDot = state.q * q_w * 0.5;
    
    for (int i = 0; i < 4; ++i) {
        state.q[i] += state.qDot[i] * dt;
    }
    state.q.normalize();
    
    return state;
}

void Rocket::update(const Input& input, double dt)
{ 
    RocketState state = Rocket::dynamics(m_state, input, dt);
    m_state = state;
    m_data.push_back(state); 
}

void Rocket::processInput(const Input& input)
{
    for (int i{ 0 }; i < m_engines.size(); i++)
    {
        m_engines[i].setGimbalAngles(input.gimbalAngles[i]);
        m_engines[i].setThrottle(input.throttle[i]);
    }
}



void Rocket::plotTrajectory(double dt) const
{
    std::vector<double> timeData(m_data.size());
    std::vector<double> xData(m_data.size());
    std::vector<double> yData(m_data.size());
    std::vector<double> zData(m_data.size());


    for (int i = 0; i < m_data.size(); ++i) {
        timeData[i] = i * dt; 
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
