#ifndef _TOY_CONTROL_H_
#define _TOY_CONTROL_H_

#include <iostream>
#include <fstream>
#include <sim/toybody.h>
#include "controller.h"

class Simulator;
class ToyControl : public Controller
{
    ToyBody& _toy;
    Simulator* _sim;
    double _ks[NUM_JOINTS];
    double _kd[NUM_JOINTS];
    double _target[NUM_JOINTS];

public:
    ToyControl(ToyBody& toy);
    ~ToyControl();
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
        _sim = simptr;
    }

    int getSimLength();
};

#endif
