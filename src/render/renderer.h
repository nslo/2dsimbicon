#include <sim/simulator.h>
#include "drawstuff.h"

class Renderer
{
    Simulator& _sim;
    int _loopCounter;
    dsFunctions _fn;

public:
    Renderer(Simulator& sim);
    void renderSim(
        int argc,
        char** argv,
        int width,
        int height
    );

    // functions to interface with drawstuff
    void start();
    void step(int pause);
    void command(int cmd);
    void stop();

    void setActive();
    static void wrapperToStart();
    static void wrapperToStep(int pause);
    static void wrapperToCommand(int cmd);
    static void wrapperToStop();

    static void* ptr2ActiveRender;
};

