#ifndef _TOY_BODY_
#define _TOY_BODY_

#include <fstream>
#include "body.h"
#include "environment.h"

/* Link and joint labeling and ordering. */
enum BODY_ORDER
{
    BODY_TORSO,
    BODY_LTHIGH,
    BODY_RTHIGH,
    BODY_LSHIN,
    BODY_RSHIN,
    BODY_LFOOT,
    BODY_RFOOT,
    BODY_END
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

const int NUM_BODY = BODY_END;
const int NUM_JOINTS = NUM_BODY - 1;

class Biped7 : public Body
{
    friend class SimbiconControl;

public:
    Biped7(Environment& env);
    void render();
    void reset();

private:
    dBody body[NUM_BODY];
    dHingeJoint joint[NUM_JOINTS];
    dJointID joint_2d_cons[NUM_BODY];
    dBox box[NUM_BODY];
    Environment& _env;

    void positionBody();
};

#endif
