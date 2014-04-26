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

/* Figure out a better structure for these at some point so they don't
 * have to be global. */
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

    /* Initial side lengths and densities of boxes. */
    //const double YSIDE = 0.2;
    const double XSIDE = 0.04;
    const double ZSIDE = 1e-6;
    const double DENSITY = 1.0;

    /* Lengths (meters) of body parts. */
    const double LENGTH_TORSO = 0.48;
    const double LENGTH_THIGH = 0.45;
    const double LENGTH_SHIN = 0.45;
    const double LENGTH_FOOT = 0.2;

    /* Masses (kilograms) of body parts. */
    const double MASS_TORSO = 70.0;
    const double MASS_THIGH = 5.0;
    const double MASS_SHIN = 4.0;
    const double MASS_FOOT = 1.0;

    /* Initial torso position. */
    const double TORSO_POS = 0.5 * LENGTH_TORSO + LENGTH_THIGH + LENGTH_SHIN + 0.5 * XSIDE;

    void positionBody();
};

#endif