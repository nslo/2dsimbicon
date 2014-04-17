#include <control/toycontrol.h>
#include <fstream>
#include <iostream>
#include <render/renderer.h>
#include <sim/simulator.h>
#include <sim/toybody.h>

const int VPX = 1024;
const int VPY = 768;

int main(int argc, char **argv)
{
    dInitODE();

    int render_steps = 10000;
    double gx = 0.0;
    double gy = -9.8;
    double gz = 0.0;

    Environment myEnv(gx, gy, gz);
    ToyBody myBody(myEnv);
    ToyControl myControl(myBody);
    Simulator mySim(myBody, myEnv, render_steps, STEP_SIZE);
    mySim.setController(&myControl);
    myControl.setSimulator(&mySim);

    Renderer myRend(mySim);
    mySim.reset();
    myRend.renderSim(argc, argv, VPX, VPY);

    return 0;
}

