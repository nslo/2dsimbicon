#ifndef _TOY_CONTROL_H_
#define _TOY_CONTROL_H_

#include <iostream>
#include <fstream>
#include <sim/biped7.h>
#include <sim/simulator.h>
#include <vector>
#include "controller.h"

enum simbicon_target_t
{
    SIMBICON_SWH,
    SIMBICON_SWK,
    SIMBICON_SWA,
    SIMBICON_STH,
    SIMBICON_STK,
    SIMBICON_STA,
    SIMBICON_TOR,
    SIMBICON_TARGET_END
};

struct Simbicon_state
{
    int id;
    int next_state;
    double duration;
    /* Gains. */
    double c_d;
    double c_v;
    /* First element in this array won't be used. */
    double target[SIMBICON_TARGET_END];
    /* The foot actually on the ground. */
    body_link_t stance_foot;
    /* The foot on which we want to detect a collision. */
    body_link_t collision_foot;
};

class SimbiconControl : public Controller
{
public:
    SimbiconControl(Biped7& _biped);
    ~SimbiconControl();
    int action();
    int getSimLength();
    void setSimulator(Simulator* simptr)
    {
        sim = simptr;
    }
    double discount()
    {
        return 1;
    }
    void reset();
    double eval(); /* Not using. */
    double norm(); /* Not using. */
    void render(); /* Not using. */

private:
    dReal get_global_angle(body_link_t link, dVector3 axis);
    void get_global_angular_vel(body_link_t link, dVector3 vel);
    void compute_torque(simbicon_target_t);
    Biped7& biped;
    Simulator* sim;
    /* The states in the state machine. */
    std::vector<Simbicon_state> states;
    int current_state;
    double start_time;
    double elapsed_time;
    double kp[SIMBICON_TARGET_END];
    double kd[SIMBICON_TARGET_END];
    double target_angle[NUM_JOINTS];
    double torque[NUM_JOINTS];
    double target_angle_limit[NUM_JOINTS][2];
    double torque_limit[SIMBICON_TARGET_END][2];
    joint_t joint_side[SIMBICON_TARGET_END];
    dReal torso_angle;
    body_link_t swing_thigh;
    dReal swing_thigh_target_angle;
    dVector3 torso_velocity;
};

#endif
