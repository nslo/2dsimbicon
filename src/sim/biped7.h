#ifndef _TOY_BODY_
#define _TOY_BODY_

#include <fstream>
#include "body.h"
#include "environment.h"

/* Link and joint labeling and ordering. */
enum body_link_t
{
    BODY_TORSO,     //0
    BODY_LTHIGH,    //1
    BODY_RTHIGH,    //2
    BODY_LSHIN,     //3
    BODY_RSHIN,     //4
    BODY_LFOOT,     //5
    BODY_RFOOT,     //6
    NUM_BODY        //7
};
enum joint_t
{
    JOINT_LHIP,     //0
    JOINT_RHIP,     //1
    JOINT_LKNEE,    //2
    JOINT_RKNEE,    //3
    JOINT_LANKLE,   //4
    JOINT_RANKLE,   //5
    NUM_JOINTS      //6
};

class Biped7 : public Body
{
friend class SimbiconControl;

public:
    Biped7(Environment& _env);
    void render();
    void reset();

private:
    void get_com(dVector3 com);
    dBody body[NUM_BODY];
    dHingeJoint joint[NUM_JOINTS];
    dJointID joint_2d_cons[NUM_BODY];
    dBox box[NUM_BODY];
    Environment& env;
    body_link_t body_draw_order[NUM_BODY];

    /* Initial side lengths and densities of boxes. */
    const double YSIDE = 0.2; /* Not actually used. */
    const double XSIDE = 0.04;
    const double ZSIDE = 1e-6; /* 2D. */
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

    /* Set up initial position. */
    void positionBody();
};

#endif
