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

enum coord_t
{
    X,
    Y,
    Z
};

dVector3 y_axis_up = {0, 1, 0};

/* Ensure that angles are given in positive clockwise coords. */
double calc_angle(double x)
{
    /* Mod by 2 pi. */
    int times = x / (2 * M_PI); 
    double mod = x - ((2 * M_PI) * times);

    /* If mod is between pi and -pi, return. */
    if (M_PI >= mod && mod > -M_PI)
    {
        return mod;
    }

    /* If mod is greater than pi, subtract 2 pi. */
    if (mod > M_PI)
    {
        return mod - (2 * M_PI);
    }

    /* If mod is less than -pi, add 2 pi. */
    if (mod < -M_PI)
    {
        return mod + (2* M_PI);
    }

    return mod;
}

double clockwise_angle(double x)
{
    if (x < 0)
    {
        x += M_PI;
    }

    return x;
}

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

dReal cos_theta(dVector3 a, dVector3 b)
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

static void clamp(int index, double *values, double limits[][2])
{
    if (values[index] < limits[index][0])
    {
        values[index] = limits[index][0];
    }
    if (values[index] > limits[index][1])
    {
        values[index] = limits[index][1];
    }
}

/* Get the angle of a body link with respect to the y-axis. */
dReal SimbiconControl::get_global_angle(body_link_t link, dVector3 axis)
{
    dVector3 link_com;
    link_com[X] = dBodyGetPosition(biped.body[link].id())[X];
    link_com[Y] = dBodyGetPosition(biped.body[link].id())[Y];
    link_com[Z] = dBodyGetPosition(biped.body[link].id())[Z];
    dVector3 hip_position;
    biped.get_com(hip_position);
    dVector3 link_vector;
    vector_subtract(link_com, hip_position, link_vector);
    dReal link_angle = std::acos(cos_theta(link_vector, axis));
    //dReal link_angle = std::asin(sin_theta(link_vector, axis));
    
    /* acos returns an angle in [0, pi], but we need a sign relative to the
     * given vector.  For now, we'll just hack it. TODO should rectify. */
    if (link_vector[0] > 0)
    {
        link_angle *= -1;
    }

    return link_angle;
}

/* Get the angular velocity of a body link about its axis of rotation. */
void SimbiconControl::get_global_angular_vel(body_link_t link, dVector3 vel)
{
    vel[X] = dBodyGetAngularVel(biped.body[link].id())[X];
    vel[Y] = dBodyGetAngularVel(biped.body[link].id())[Y];
    vel[Z] = dBodyGetAngularVel(biped.body[link].id())[Z];
}

/* Constructor. */
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
    swing_thigh = BODY_LTHIGH;

    for (int i = 0; i < NUM_SIMBICON_TARGETS; ++i)
    {
        kp[i] = 300.0;
        kd[i] = 30.0;
        torque_limit[i][0] = -400;
        torque_limit[i][1] = 400;
    }

    target_angle_limit[JOINT_LHIP][0] = -M_PI;
    target_angle_limit[JOINT_LHIP][1] = M_PI;
    target_angle_limit[JOINT_RHIP][0] = -M_PI;
    target_angle_limit[JOINT_RHIP][1] = M_PI;
    target_angle_limit[JOINT_LKNEE][0] = 0;
    target_angle_limit[JOINT_LKNEE][1] = M_PI;
    target_angle_limit[JOINT_RKNEE][0] = 0;
    target_angle_limit[JOINT_RKNEE][1] = M_PI;
    target_angle_limit[JOINT_LANKLE][0] = M_PI / 4.0;
    target_angle_limit[JOINT_LANKLE][1] = M_PI;
    target_angle_limit[JOINT_RANKLE][0] = M_PI / 4.0;
    target_angle_limit[JOINT_RANKLE][1] = M_PI;
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
                //printf("colliding foot %d\n", foot);
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
    torso_angle = get_global_angle(BODY_TORSO, y_axis_up);
    get_global_angular_vel(BODY_TORSO, torso_velocity);

    /* Swing hip: apply gains. */
    dVector3 hip_position;
    biped.get_com(hip_position);
    dVector3 stance_ankle_pos;
    dJointGetHingeAnchor(biped.joint[joint_side[SIMBICON_STA]], stance_ankle_pos);
    dVector3 hip_vel;
    dBodyGetPointVel(biped.body[BODY_TORSO].id(), hip_position[X], hip_position[Y],
            hip_position[Z], hip_vel);
    double d = hip_position[X] - stance_ankle_pos[X];
    double v = hip_vel[X];
    swing_thigh_target_angle = states[current_state].target[SIMBICON_SWH] +
        d * states[current_state].c_d + v * states[current_state].c_v;
    target_angle[joint_side[SIMBICON_SWH]] = swing_thigh_target_angle; 

    //printf("time: %f, state: %d\n", sim->currentTime(), current_state);
    //printf("torso_angle: %f, torso_velocity: %f\n",
    //        torso_angle, torso_velocity[Z]);

    /* Compute and apply torques. */
    compute_torque(SIMBICON_SWK);
    compute_torque(SIMBICON_SWA);
    compute_torque(SIMBICON_STK);
    compute_torque(SIMBICON_STA);
    compute_torque(SIMBICON_SWH);
    compute_torque(SIMBICON_STH);

    //printf("\n");

    return 0;
}

void SimbiconControl::compute_torque(simbicon_target_t simbicon_joint)
{
    joint_t j = joint_side[simbicon_joint];
    dJointID jt = biped.joint[j].id();
    dReal theta;
    dReal thetav;

    switch (simbicon_joint)
    {
        case SIMBICON_SWK:
        case SIMBICON_SWA:
        case SIMBICON_STK:
        case SIMBICON_STA:
        {
            /* Get control of non-hip joints from state machine. */
            theta = dJointGetHingeAngle(jt);
            thetav = dJointGetHingeAngleRate(jt);
            torque[j] = kp[j] * (target_angle[j] - theta) - kd[j] * thetav;

            break;
        }
        case SIMBICON_SWH:
        {
            theta = dJointGetHingeAngle(jt) + torso_angle;
            thetav = dJointGetHingeAngleRate(jt) + torso_velocity[Z];
            torque[j] = kp[j] * (target_angle[j] - theta) - kd[j] * thetav;
            break;
        }
        case SIMBICON_STH:
        {
            /* First calcuate virtual pd control torque of torso. */
            target_angle[j] = states[current_state].target[SIMBICON_TOR];
            theta = torso_angle;
            thetav = torso_velocity[Z]; /* Assume 2D. */
            /* Calculate angle and velocity of swing thigh wrt global y-axis.
             * This is accomplished just be adding the torso values. */
            dReal torso_torque =
                kp[SIMBICON_TOR] * (target_angle[j] - theta) -
                kd[SIMBICON_TOR] * thetav;

            /* Then calculate stance hip control as a function of torso torque
             * and swing hip torque. */
            torque[j] = -1.0 * torso_torque - torque[joint_side[SIMBICON_SWH]];
            break;
        }
        default:
            /* Shouldn't get here. */
            assert(0);
            break;
    }
    
    clamp(j, torque, torque_limit);
    dJointAddHingeTorque(jt, torque[j]);

    //printf("%d angle: %f, target_angle: %f, velocity: %f, torque: %f\n",
    //        j, theta, target_angle[j], thetav, torque[j]);
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
