#include <iostream>
#include "drawstuff.h"
#include "renderer.h"

#define UNUSED(expr) do { (void)(expr); } while (0)

static float xyz[3];
static float hpr[3];
static dVector3(com);

Renderer::Renderer(Simulator& sim) : _sim(sim)
{
    _fn.version = DS_VERSION;
    _fn.start = &Renderer::wrapperToStart;
    _fn.step = &Renderer::wrapperToStep;
    _fn.command = 0;
    _fn.stop = &Renderer::wrapperToStop;
    _fn.path_to_textures = (char *)"textures";
    setActive();
}

void Renderer::setActive()
{
    Renderer::ptr2ActiveRender = (void*)this;
}

void Renderer::start()
{
    float xyz[3] = {0.0f, 0.0f, 5.0f};
    float hpr[3] = {.0f, -90.0f, 90.0f};
    //float xyz[3] = {2.164f,-1.3079f,1.76f};
    //float hpr[3] = {125.5f,-17.0f,0.0f};
    _loopCounter = 0;
    dsSetViewpoint(xyz, hpr);
}

void Renderer::step(int pause)
{
    _sim.render();
    if (++_loopCounter == _sim._timeSteps)
    {
        dsStop();
        return;
    }
    if (!pause)
    {
        /* Hack to update the camera. */
        dsGetViewpoint(xyz, hpr);
        _sim._body.get_com(com);
        xyz[0] = com[0];
        dsSetViewpoint(xyz, hpr);
        _sim.simStep();
    }
}

void Renderer::command(int cmd)
{
    UNUSED(cmd);
}

void Renderer::stop() {}

void Renderer::renderSim(
    int argc, char** argv, int width, int height)
{
    _sim.setActive();
    dsSimulationLoop(argc, argv, width, height, &_fn);
}

void Renderer::wrapperToStart()
{
    Renderer* myself = (Renderer*)ptr2ActiveRender;

    myself->start();
}

void Renderer::wrapperToStep(int pause)
{
    Renderer* myself = (Renderer*)ptr2ActiveRender;

    myself->step(pause);
}

void Renderer::wrapperToCommand(int cmd)
{
    Renderer* myself = (Renderer*)ptr2ActiveRender;

    myself->command(cmd);
}

void Renderer::wrapperToStop()
{
    Renderer* myself = (Renderer*)ptr2ActiveRender;

    myself->stop();
}

void* Renderer::ptr2ActiveRender = 0;


