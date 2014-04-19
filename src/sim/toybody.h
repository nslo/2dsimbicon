#ifndef _TOY_BODY_
#define _TOY_BODY_

#include <fstream>
#include "body.h"
#include "environment.h"

const int NUM_BODY = 7;
const int NUM_JOINTS = NUM_BODY - 1;

// ID ordering
enum BODY_ORDER
{
    BODY_TORSO,
    BODY_LTHIGH,
    BODY_RTHIGH,
    BODY_LSHIN,
    BODY_RSHIN,
    BODY_LFOOT,
    BODY_RFOOT
};
enum JOINT_ORDER
{
    JOINT_LHIP,
    JOINT_RHIP,
    JOINT_LKNEE,
    JOINT_RKNEE,
    JOINT_LANKLE,
    JOINT_RANKLE
};

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
