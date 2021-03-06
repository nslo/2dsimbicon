#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_

#include <control/controller.h>
#include <vector>
#include "body.h"
#include "environment.h"

const int NUM_CONTACTP = 3;

class Simulator
{
friend class Renderer;

public:
    Simulator(Body& body, Environment& env, int totalSteps,
              double stepSize, Controller* control);
    Simulator(Body& body, Environment& env, int totalSteps,
              double stepSize);
    ~Simulator();

    void setTimeSteps(int steps);
    //double simulate( Controller* id );
    double simulate();
    double simulate(int numSteps);
    void render();
  	//void resetRandomSeed();
    void reset();
    double currentTime(); // Current simulation time in sec.
    double stepSize();
    void nearCallback(void* data, dGeomID o1, dGeomID o2);
    void enableContact(bool flag);
    void setController(Controller* control);
    Body& getBody()
    {
        return _body;
    }
    Environment& getEnv()
    {
        return _env;
    }
    void setMu(double mu);
    void setSlope(double degree);
    double getSlope();
    //void setRandomize(bool random);
    //bool getRandomize();
    //gsl_rng* getRNG()
    //{
    //	return _r;
    //}

    /* Evil hack for ode callbacks. */
    static void wrapperToCallback(void* data, dGeomID o1, dGeomID o2);
    static void* ptr2ActiveSim;

private:
    Environment& _env;
    Body& _body;
    int _timeSteps;
    double _stepSize;
    double _currentTime;
    dJointGroup _contactGroup;
    Controller* _controller;
    bool _enableContact;
    double _mu;
    bool _randomize;
    void simStep();
    void setActive();
    //const gsl_rng_type * _T;
    //gsl_rng * _r;
};

#endif
