#include "Engine.h"



Vector3D Engine::getThrust()
{
    return Vector3D{ 0.0, 0.0, m_maxThrust * m_throttle };
}

Matrix Engine::getRengine2rocket()
{
    double c1 = cos(m_gimbalAngle[0]); //yaw
    double s1 = sin(m_gimbalAngle[0]);
    double c2 = cos(m_gimbalAngle[1]); //pitch
    double s2 = sin(m_gimbalAngle[1]);

    Matrix R(3, 3);

    R[0][0] = c2;
    R[0][1] = s1 * s2;
    R[0][2] = s2 * c1;

    R[1][0] = 0.0;
    R[1][1] = c1;
    R[1][2] = -s1;

    R[2][0] = -s2;
    R[2][1] = c2 * s1;
    R[2][2] = c2 * c1;
    return R;
}

void Engine::setGimbalAngles(const std::vector<double>& angles) {
    for (int i = 0; i < 2; ++i) {
        // Check if the angle is within the allowed range
        if (angles[i] < -m_maxGimbalAngle || angles[i] > m_maxGimbalAngle) {
            throw std::invalid_argument("Gimbal angle out of range.");
        }
    }
    m_gimbalAngle = angles;
}
