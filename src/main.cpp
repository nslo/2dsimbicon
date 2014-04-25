#include <control/simbicon_control.h>
#include <fstream>
#include <iostream>
#include <render/renderer.h>
#include <sim/simulator.h>
#include <sim/biped7.h>

int main(int argc, char **argv)
{
    dInitODE();

    const int VPX = 1024;
    const int VPY = 768;
    const int render_steps = 10000;
    const double gx = 0.0;
    const double gy = -9.8;
    const double gz = 0.0;

    Environment myEnv(gx, gy, gz);
    Biped7 myBody(myEnv);
    SimbiconControl myControl(myBody);
    Simulator mySim(myBody, myEnv, render_steps, STEP_SIZE);
    mySim.setController(&myControl);
    myControl.setSimulator(&mySim);

    Renderer myRend(mySim);
    mySim.reset();
    myRend.renderSim(argc, argv, VPX, VPY);

    return 0;
}

