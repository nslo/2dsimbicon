#ifndef _TOY_CONTROL_H_
#define _TOY_CONTROL_H_

#include <iostream>
#include <fstream>
#include <sim/biped7.h>
#include <sim/simulator.h>
#include <vector>
#include "controller.h"

enum simbicon_targets
{
    SIMBICON_TOR,
    SIMBICON_SWH,
    SIMBICON_SWK,
    SIMBICON_SWA,
    SIMBICON_STK,
    SIMBICON_STA,
    SIMBICON_TARGET_END
};

struct Simbicon_state
{
    int id;
    int next_state;
    double duration;
    double c_d;
    double c_v;
    double target[SIMBICON_TARGET_END];
    BODY_ORDER stance_foot;
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
    Biped7& biped;
    Simulator* sim;
    /* The states in the state machine. */
    std::vector<Simbicon_state> states;
    int current_state;
    double start_time;
    double elapsed_time;
    BODY_ORDER current_stance;
    static const int num_targets = SIMBICON_TARGET_END;
    /* The below is for the current step. */
    double kp[NUM_JOINTS];
    double kd[NUM_JOINTS];
    double target[NUM_JOINTS];
    double joint_limit[NUM_JOINTS];
    double torque_limit[NUM_JOINTS];
};

#endif
