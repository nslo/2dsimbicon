#ifndef _BODY_
#define _BODY_

#include <ode/ode.h>

class Body
{
public:
    virtual ~Body() {};
    virtual void render() = 0;
    virtual void reset() = 0;
};

#endif
