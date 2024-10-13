#include "Engine.h"



Vector3D Engine::getThrust()
{
    return Vector3D{ 0.0, 0.0, m_maxThrust * m_throttle };
}

Matrix3x3 Engine::getRengine2rocket()
{
    double c1 = cos(m_gimbalAngle[0]); //yaw
    double s1 = sin(m_gimbalAngle[0]);
    double c2 = cos(m_gimbalAngle[1]); //pitch
    double s2 = sin(m_gimbalAngle[1]);

    Matrix3x3 R = { {
        {  c2,   s1 * s2,   s2 * c1 },
        {  0.0,  c1,        -s1},
        { -s2,   c2 * s1,   c2 * c1}
    } };
    return R;
}