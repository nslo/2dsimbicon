#ifndef _TOY_CONTROL_H_
#define _TOY_CONTROL_H_

#include <iostream>
#include <fstream>
#include <sim/biped7.h>
#include <sim/simulator.h>
#include "controller.h"

struct Simbicon_state
{
    int id;
    int next_id;
    double duration;
    double target_joint_angle[NUM_JOINTS];
    BODY_ORDER stance_foot;
};

class SimbiconControl : public Controller
{
public:
    SimbiconControl(Biped7& toy);
    ~SimbiconControl();
    int action();
    double eval();
    double discount()
    {
        return 1;
    }
    double norm();
    void render();
    void reset();
    void setSimulator(Simulator* simptr)
    {
        sim = simptr;
    }
    int getSimLength();

private:
    Biped7& toy;
    Simulator* sim;
    double kp[NUM_JOINTS];
    double kd[NUM_JOINTS];
    double target[NUM_JOINTS];
    double joint_limit[NUM_JOINTS];
    double torque_limit[NUM_JOINTS];
    static const int num_states = 4;
    Simbicon_state states[num_states];
    int current_state;
    double elapsed_time;
};

#endif
