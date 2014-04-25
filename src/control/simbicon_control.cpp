#include <cassert>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <render/drawstuff.h>
#include <sim/simulator.h>
#include <sim/biped7.h>
#include "simbicon_control.h"

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawLine dsDrawLineD
#endif

SimbiconControl::SimbiconControl(Biped7& human)
    : toy(human)
{
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        kp[i] = 300.0;  // kp in simbicon paper
        kd[i] = 30.0;
        switch(i)
        {
            case JOINT_LHIP:
                target[i] = -1;
                break;
            case JOINT_RHIP:
                target[i] = 0.5;
                break;
            case JOINT_LKNEE:
                target[i] = 1;
                break;
            case JOINT_RKNEE:
                target[i] = 0;
                break;
            case JOINT_LANKLE:
                target[i] = 0;
                break;
            case JOINT_RANKLE:
                target[i] = 0;
                break; 
            default:
                assert(0);
                break;
        }
    }
}

SimbiconControl::~SimbiconControl()
{
}

int SimbiconControl::action()
{
    if (!sim)
    {
        return -1;
    }

    // Add joint torques to each DOF, pulling the body towards the
    // desired state defined by _target.
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        dJointID jt = toy.joint[i].id();
        double limit = kp[i];

        // angles are in radians
        dReal theta = dJointGetHingeAngle(jt);
        dReal thetav = dJointGetHingeAngleRate(jt);
        dReal torque =
            kp[i] * (target[i] - theta) - kd[i] * thetav;

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

double SimbiconControl::eval()
{
    return 0.0;
}

double SimbiconControl::norm()
{
    return 0.0;
}

void SimbiconControl::reset()
{
}

void SimbiconControl::render()
{
}
