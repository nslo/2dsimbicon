#ifndef _BODY_
#define _BODY_

#include <ode/ode.h>

class Body
{
friend class Renderer;
friend class Simulator;

public:
    virtual ~Body() {};
    virtual void render() = 0;
    virtual void reset() = 0;

private:
    virtual void get_com(dVector3) = 0;
    virtual void get_torso_pos(dVector3) = 0;
    virtual void apply_push(dVector3) = 0;
};

#endif
