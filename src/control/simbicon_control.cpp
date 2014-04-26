#include <cassert>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <render/drawstuff.h>
#include <sim/biped7.h>
#include <sim/simulator.h>
#include "simbicon_control.h"

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawLine dsDrawLineD
#endif

//const double epsilon = 0.00001;

enum coord_t
{
    X,
    Y,
    Z
};

/* Quick, dirty, and unsafe vector functions. */
static void vector_subtract(dVector3 a, dVector3 b, dVector3 result)
{
    result[0] = a[0] - b[0];
    result[1] = a[1] - b[1];
    result[2] = a[2] - b[2];
}

static dReal vector_length(dVector3 v)
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static dReal vector_dot(dVector3 a, dVector3 b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static dReal cos_theta(dVector3 a, dVector3 b)
{
    return vector_dot(a, b) / (vector_length(a) * vector_length(b));
}

static void vector_cross(dVector3 a, dVector3 b, dVector3 result)
{
    dCROSS(result, =, a, b); 
}

dReal sin_theta(dVector3 a, dVector3 b)
{
    dVector3 result;
    vector_cross(a, b, result);
    return vector_length(result) / (vector_length(a) * vector_length(b));
}

dReal SimbiconControl::get_global_angle(body_link_t link)
{
    dVector3 link_com;
    link_com[X] = dBodyGetPosition(biped.body[link].id())[X];
    link_com[Y] = dBodyGetPosition(biped.body[link].id())[Y];
    link_com[Z] = dBodyGetPosition(biped.body[link].id())[Z];
    dVector3 hip_position;
    /* Shouldn't matter which hip we use for reference position. */
    dJointGetHingeAnchor(biped.joint[JOINT_LHIP], hip_position);
    dVector3 link_vector;
    vector_subtract(link_com, hip_position, link_vector);
    dVector3 y_axis;
    y_axis[X] = 0;
    y_axis[Y] = 1;
    y_axis[Z] = 0;
    dReal link_angle = std::acos(cos_theta(link_vector, y_axis));
    /* We're just going to hack the sign of the angle. */
    if (link_vector[0] < 0)
    {
        link_angle *= -1;
    }

    return link_angle;
}

void SimbiconControl::get_global_angular_vel(body_link_t link, dVector3 vel)
{
    vel[X] = dBodyGetAngularVel(biped.body[link].id())[X];
    vel[Y] = dBodyGetAngularVel(biped.body[link].id())[Y];
    vel[Z] = dBodyGetAngularVel(biped.body[link].id())[Z];

}



SimbiconControl::SimbiconControl(Biped7& _biped) : biped(_biped)
{
    /* Set up the four states here. */
    Simbicon_state state0;
    state0.id = 0;
    state0.next_state = 1;
    state0.duration = 0.3; /* 0.3s TODO relate this to stepsize */
    state0.c_d = 0.0;
    state0.c_v = 0.20;
    state0.collision_foot = BODY_RFOOT;
    state0.stance_foot = BODY_RFOOT;
    state0.target[SIMBICON_TOR] = 0.0;      /* WRT world. */
    state0.target[SIMBICON_SWH] = 0.40;     /* WRT world. */
    state0.target[SIMBICON_SWK] = -1.10;    /* Local. */
    state0.target[SIMBICON_SWA] = 0.20;     /* Local. */
    state0.target[SIMBICON_STK] = -0.05;    /* Local. */
    state0.target[SIMBICON_STA] = 0.20;     /* Local. */
    states.push_back(state0);

    Simbicon_state state1;
    state1.id = 1;
    state1.next_state = 2;
    state1.duration = 0.3; /* Actually will transition on contact. */
    state1.c_d = 2.20;
    state1.c_v = 0.0;
    state1.collision_foot = BODY_LFOOT;
    state1.stance_foot = BODY_RFOOT;
    state1.target[SIMBICON_TOR] = 0.0;      /* WRT world. */
    state1.target[SIMBICON_SWH] = -0.70;    /* WRT world. */
    state1.target[SIMBICON_SWK] = -0.05;    /* Local. */
    state1.target[SIMBICON_SWA] = 0.20;     /* Local. */
    state1.target[SIMBICON_STK] = -0.10;    /* Local. */
    state1.target[SIMBICON_STA] = 0.20;     /* Local. */
    states.push_back(state1);

    /* The other states are mirror images of the first two. */
    Simbicon_state state2 = state0;
    state2.collision_foot = BODY_LFOOT;
    state2.stance_foot = BODY_LFOOT;
    state2.next_state = 3;
    states.push_back(state2);

    Simbicon_state state3 = state1;
    state3.collision_foot = BODY_RFOOT;
    state3.stance_foot = BODY_LFOOT;
    state3.next_state = 0;
    states.push_back(state3);

    /* Start off on the right foot. */
    current_state = 0;
    start_time = 0;
    elapsed_time = 0;

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        kp[i] = 300.0;
        kd[i] = 30.0;
        torque_limit[i] = 1000;
        switch(i)
        {
            case JOINT_LHIP:
                target_angle[i] = -1;
                break;
            case JOINT_RHIP:
                target_angle[i] = 0.5;
                break;
            case JOINT_LKNEE:
                target_angle[i] = 1;
                break;
            case JOINT_RKNEE:
                target_angle[i] = 0;
                break;
            case JOINT_LANKLE:
                target_angle[i] = 0;
                break;
            case JOINT_RANKLE:
                target_angle[i] = 0;
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

    elapsed_time = sim->currentTime() - start_time;

    printf("state: %d, elapsed_time: %f, stance_foot Y: %f, left_foot Y: %f, right_foot Y: %f\n",
            current_state, elapsed_time,
            biped.body[states[current_state].stance_foot].getLinearVel()[Y],
            biped.body[BODY_LFOOT].getLinearVel()[Y],
            biped.body[BODY_RFOOT].getLinearVel()[Y]);

    /* Check for transitions. */
    switch (current_state)
    {
        case 0:
        case 2:
            {
                /* Check for an elapsed time transition. */
                if (elapsed_time > states[current_state].duration)
                {
                    current_state = states[current_state].next_state;
                    start_time = sim->currentTime();
                }
                break;
            }
        case 1:
        case 3:
            {
                /* Check for a contact transition. */
                body_link_t foot = states[current_state].collision_foot;
                dContactGeom temp_contacts[NUM_CONTACTP];
                if (dCollide(sim->getEnv().ground.id(), biped.box[foot].id(),
                            NUM_CONTACTP, temp_contacts, sizeof(dContactGeom)))
                {
                    printf("colliding foot %d\n", foot);
                    current_state = states[current_state].next_state;
                    start_time = sim->currentTime();
                }
                break;
            }
        default:
            /* Should never get here. */
            assert(0);
            break;
    }

    /* Get stance. */
    joint_t joint_side[SIMBICON_TARGET_END];
    body_link_t foot = states[current_state].stance_foot;
    switch (foot)
    {
        case BODY_LFOOT:
            joint_side[SIMBICON_SWH] = JOINT_RHIP;
            joint_side[SIMBICON_SWK] = JOINT_RKNEE;
            joint_side[SIMBICON_SWA] = JOINT_RANKLE;
            joint_side[SIMBICON_STH] = JOINT_LHIP;
            joint_side[SIMBICON_STK] = JOINT_LKNEE;
            joint_side[SIMBICON_STA] = JOINT_LANKLE;
            swing_thigh = BODY_RTHIGH;
            /* Left foot on the ground. */
            break;
        case BODY_RFOOT:
            /* Right foot on the ground. */
            joint_side[SIMBICON_SWH] = JOINT_LHIP;
            joint_side[SIMBICON_SWK] = JOINT_LKNEE;
            joint_side[SIMBICON_SWA] = JOINT_LANKLE;
            joint_side[SIMBICON_STH] = JOINT_RHIP;
            joint_side[SIMBICON_STK] = JOINT_RKNEE;
            joint_side[SIMBICON_STA] = JOINT_RANKLE;
            swing_thigh = BODY_LTHIGH;
            break;
        default:
            /* Should never get here. */
            assert(0);
            break;
    }

    /* Set joint targets appropriately. For joints other than hips, the target
     * angle comes from the FSM. */
    target_angle[joint_side[SIMBICON_SWK]] =
        states[current_state].target[SIMBICON_SWK];
    target_angle[joint_side[SIMBICON_SWA]] =
        states[current_state].target[SIMBICON_SWA];
    target_angle[joint_side[SIMBICON_STK]] =
        states[current_state].target[SIMBICON_STK];
    target_angle[joint_side[SIMBICON_STA]] =
        states[current_state].target[SIMBICON_STA];

    /* Stance hip: calculate angle and velocity of torso wrt global y-axis. */
    dReal torso_angle = get_global_angle(BODY_TORSO);
    dVector3 torso_velocity;
    get_global_angular_vel(BODY_TORSO, torso_velocity);

    /* Swing hip: calculate angle and velocity of swing thigh wrt global y-axis. */
    dReal swing_thigh_angle = get_global_angle(swing_thigh);
    dVector3 swing_thigh_velocity;
    get_global_angular_vel(swing_thigh, swing_thigh_velocity);

    /* Swing hip: apply gains. */
    dVector3 hip_position;
    dVector3 stance_ankle_pos;
    dVector3 hip_vel;
    /* Shouldn't matter which hip we use for reference position. */
    dJointGetHingeAnchor(biped.joint[JOINT_LHIP], hip_position);
    dJointGetHingeAnchor(biped.joint[joint_side[SIMBICON_STA]], stance_ankle_pos);
    dBodyGetPointVel(biped.body[BODY_TORSO].id(), hip_position[X], hip_position[Y],
            hip_position[Z], hip_vel);
    double d = hip_position[X] - stance_ankle_pos[X];
    double v = hip_vel[X];
    double swing_theta = states[current_state].target[SIMBICON_SWH] +
        d * states[current_state].c_d + v * states[current_state].c_v;

    printf("torso_angle: %f, swing_thigh_angle: %f, swing_theta: %f\n",
            torso_angle, swing_thigh_angle, swing_theta);





    /* Add joint torques to each DOF, pulling the body towards the
     * desired state defined by _target. */
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        dJointID jt = biped.joint[i].id();
        double limit = kp[i];

        /* Angles are in radians. */
        dReal theta = dJointGetHingeAngle(jt);
        dReal thetav = dJointGetHingeAngleRate(jt);
        dReal torque = kp[i] * (target_angle[i] - theta) - kd[i] * thetav;

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
