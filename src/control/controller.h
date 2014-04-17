#ifndef _CONTROLLER_
#define _CONTROLLER_

class Controller
{
public:
    virtual ~Controller() {};
    virtual int action() = 0;
    virtual double eval() = 0;
    virtual double discount() = 0;
    virtual double norm() = 0;
    virtual void render() = 0;
    virtual void reset() = 0;
};
#endif
