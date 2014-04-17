#ifndef _TOY_BODY_
#define _TOY_BODY_

#include <fstream>
#include "body.h"
#include "environment.h"

const int NUM_BODY = 7;
const int NUM_JOINTS = NUM_BODY - 1;

class ToyBody : public Body
{
    friend class ToyControl;
    dBody body[NUM_BODY];
    dHingeJoint joint[NUM_JOINTS];
    dJointID joint_2d_cons[NUM_BODY];
    dBox box[NUM_BODY];
    Environment& _env;

    void positionBody();
public:
    ToyBody(Environment& env);
    void render();
    void reset();
};

#endif
