#include <cfloat>
#include <ctime>
#include <iostream>
#include "simulator.h"

#define UNUSED(expr) do { (void)(expr); } while (0)

double slope_value; // this variable decides the ground rendering

Simulator::Simulator(Body& body, Environment& env, int totalSteps,
                     double stepSize, Controller* control) :
    _env(env), _body(body), _timeSteps(totalSteps),
    _stepSize(stepSize), _currentTime(0), _controller(control),
    _enableContact(true), _mu(0.8), _randomize(false)
{
    /*
    	gsl_rng_env_setup();
    	_T = gsl_rng_default;
    	_r = gsl_rng_alloc(_T);
    	gsl_rng_set(_r, 0);
    */

}

Simulator::Simulator(Body& body, Environment& env, int totalSteps,
                     double stepSize) :
    _env(env), _body(body), _timeSteps(totalSteps),
    _stepSize(stepSize), _currentTime(0), _controller(NULL),
    _enableContact(true), _mu(0.8), _randomize(false)
{
    /*
    	gsl_rng_env_setup();
    	_T = gsl_rng_default;
    	_r = gsl_rng_alloc(_T);
    	gsl_rng_set(_r, 0);
    */
}

Simulator::~Simulator()
{
//	gsl_rng_free(_r);
}

void Simulator::enableContact(bool flag)
{
    _enableContact = flag;
}

void Simulator::setActive()
{
    Simulator::ptr2ActiveSim = (void *)this;
}

void Simulator::setTimeSteps(int steps)
{
    _timeSteps = steps;
}

double Simulator::simulate(int numSteps)
{
    setActive();
    double value = 0;
    double factor = 0;

    if (_controller)
    {
        factor = _controller->discount();
    }

//	clock_t et = clock();
    for (int i = 0; i < numSteps; i++)
    {
        double reward = 0;
        if (_controller)
        {
            reward = _controller->eval();
        }
        if (isnan(reward))
        {
            return HUGE_VAL;
        }

        value += factor * reward;

        if (i == numSteps - 1)
        {
            break;
        }

        simStep();
    }
//	et = clock() - et;
//	std::cout << double(et)/CLOCKS_PER_SEC << std::endl;

    if (isnan(value))
    {
        return HUGE_VAL;
    }

    if (_controller)
    {
        return -value + _controller->norm();
    }
    else
    {
        return -value;
    }
    return 0;
}

double Simulator::simulate()
{
    return simulate(_timeSteps);
}

void Simulator::render()
{
    _body.render();
    if (_controller)
        _controller->render();
}

void Simulator::simStep()
{
    if (_controller)
    {
        _controller->action();
    }
    _env.space.collide(0, &Simulator::wrapperToCallback);
    _env.world.step(_stepSize);

    _currentTime += _stepSize;
    _contactGroup.empty();
}
/*
void Simulator::resetRandomSeed() {
	gsl_rng_set(_r, 0);
}*/

void Simulator::reset()
{
    _body.reset();
    if (_controller)
    {
        _controller->reset();
    }
    _currentTime = 0;
}

double Simulator::currentTime()
{
    return _currentTime;
}

double Simulator::stepSize()
{
    return _stepSize;
}

void Simulator::setMu(double mu)
{
    _mu = mu;
}

void Simulator::setSlope(double degree)
{
    slope_value = degree;
    _env.setSlope(degree);
}

double Simulator::getSlope()
{
    return _env.getSlope();
}
/*
void Simulator::setRandomize(bool random) {
	_randomize = random;
}

bool Simulator::getRandomize() {
	return _randomize;
}*/

void Simulator::wrapperToCallback(
    void* data, dGeomID o1, dGeomID o2)
{
    Simulator* myself = (Simulator *)ptr2ActiveSim;

    myself->nearCallback(data, o1, o2);
}

void Simulator::nearCallback(
    void* data, dGeomID o1, dGeomID o2)
{
    UNUSED(data);

    if (!_enableContact) return;

    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);

    if (b1 && b2) return;
    //if (b1 && b2 && dAreConnected (b1,b2)) return;

    dContact contact[NUM_CONTACTP];
    for (int i = 0; i < NUM_CONTACTP; i++)
    {
        contact[i].surface.mode = dContactApprox1;
        contact[i].surface.mu = _mu;
    }

    int numc = dCollide(
                   o1, o2, NUM_CONTACTP, &contact[0].geom, sizeof(dContact));

    if (numc > 0)
    {
        for (int i = 0; i < numc; i++)
        {
            dJointID c = dJointCreateContact(_env.world.id(),
                                             _contactGroup.id(), contact + i);
            dJointAttach(c, b1, b2);
        }
    }
}

void Simulator::setController(Controller* control)
{
    _controller = control;
}

void* Simulator::ptr2ActiveSim = 0;
