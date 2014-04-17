#ifndef _SIMULATOR_
#define _SIMULATOR_

#include <control/controller.h>
#include <vector>
#include "body.h"
#include "environment.h"

#define SPS (1000)
#define STEP_SIZE (1.0 / SPS)
#define NUM_CONTACTP 3

class Simulator
{
    friend class Renderer;
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
    /*
    	const gsl_rng_type * _T;
    	gsl_rng * _r;
    */
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
//	void resetRandomSeed();
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

    void setMu(double mu);
    void setSlope(double degree);
    double getSlope();
    /*
    	void setRandomize(bool random);
    	bool getRandomize();


    	gsl_rng* getRNG() {
    		return _r;
    	}
    	*/

    // evil hack for ode callbacks
    static void wrapperToCallback(
        void* data, dGeomID o1, dGeomID o2);
    static void* ptr2ActiveSim;
};

#endif
