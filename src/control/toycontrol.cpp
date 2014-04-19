#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <render/drawstuff.h>
#include <sim/simulator.h>
#include <sim/toybody.h>
#include "toycontrol.h"

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawLine dsDrawLineD
#endif

ToyControl::ToyControl(ToyBody& human)
    : _toy(human)
{
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        _ks[i] = 300.0;  // kp in simbicon paper
        _kd[i] = 30.0;
        _target[i] = 0;
    }

    _target[0] = -1;
    _target[1] = 0.5;
    _target[2] = 1;
}

ToyControl::~ToyControl()
{
}

int ToyControl::action()
{
    if (!_sim)
    {
        return -1;
    }

    // Add joint torques to each DOF, pulling the body towards the
    // desired state defined by _target.
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        dJointID jt = _toy.joint[i].id();
        double limit = _ks[i];

        // angles are in radians
        dReal theta = dJointGetHingeAngle(jt);
        dReal thetav = dJointGetHingeAngleRate(jt);
        dReal torque =
            _ks[i] * (_target[i] - theta) - _kd[i] * thetav;

        if (torque > limit)
        {
            torque = limit;
        }
        if (torque < -limit)
        {
            torque = -limit;
        }

        dJointAddHingeTorque(jt, torque);
    }

    return 0;
}

double ToyControl::eval()
{
    return 0.0;
}

double ToyControl::norm()
{
    return 0.0;
}

void ToyControl::reset()
{
}

void ToyControl::render()
{
}
