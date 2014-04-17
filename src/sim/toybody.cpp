#include <iostream>
#include <render/drawstuff.h>
#include "simulator.h"
#include "toybody.h"

// ODE convenience
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

// initial side lengths and densities of boxes
//const double YSIDE = 0.2;
const double XSIDE = 0.04;
const double ZSIDE = 1e-6;
const double DENSITY = 1.0;

// lengths (meters) of body parts
const double LENGTH_TORSO = 0.48;
const double LENGTH_THIGH = 0.45;
const double LENGTH_SHIN = 0.4;
const double LENGTH_FOOT = 0.2;

// masses (kilograms) of body parts
const double MASS_TORSO = 7.0;
const double MASS_THIGH = 5.0;
const double MASS_SHIN = 5.0;
const double MASS_FOOT = 1.0;

// initial torso position
const double TORSO_POS = 0.5 * LENGTH_TORSO + LENGTH_THIGH + LENGTH_SHIN + 0.5 * XSIDE;

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

static const int drawOrder[NUM_BODY] = {
    BODY_LTHIGH, BODY_LSHIN, BODY_LFOOT,
    BODY_TORSO,
    BODY_RTHIGH, BODY_RSHIN, BODY_RFOOT};

void ToyBody::positionBody()
{
    dMass m;

    // torso
    body[BODY_TORSO].setPosition(0, TORSO_POS, 0) ;
    m.setBox(DENSITY, XSIDE, LENGTH_TORSO, ZSIDE);
    m.adjust(MASS_TORSO);
    body[BODY_TORSO].setMass(&m);
    box[BODY_TORSO].setBody(body[BODY_TORSO]);
    // left thigh
    body[BODY_LTHIGH].setPosition(0, TORSO_POS - 0.5 * LENGTH_TORSO - 0.5 * LENGTH_THIGH, 0) ;
    m.setBox(DENSITY, XSIDE, LENGTH_THIGH, ZSIDE);
    m.adjust(MASS_THIGH);
    body[BODY_LTHIGH].setMass(&m);
    box[BODY_LTHIGH].setBody(body[BODY_LTHIGH]);
    // right thigh
    body[BODY_RTHIGH].setPosition(0, TORSO_POS - 0.5 * LENGTH_TORSO - 0.5 * LENGTH_THIGH, 0) ;
    m.setBox(DENSITY, XSIDE, LENGTH_THIGH, ZSIDE);
    m.adjust(MASS_THIGH);
    body[BODY_RTHIGH].setMass(&m);
    box[BODY_RTHIGH].setBody(body[BODY_RTHIGH]);
    // left shin
    body[BODY_LSHIN].setPosition(0, TORSO_POS - 0.5 * LENGTH_TORSO - LENGTH_THIGH - 0.5 * LENGTH_SHIN, 0) ;
    m.setBox(DENSITY, XSIDE, LENGTH_SHIN, ZSIDE);
    m.adjust(MASS_SHIN);
    body[BODY_LSHIN].setMass(&m);
    box[BODY_LSHIN].setBody(body[BODY_LSHIN]);
    // right shin
    body[BODY_RSHIN].setPosition(0, TORSO_POS - 0.5 * LENGTH_TORSO - LENGTH_THIGH - 0.5 * LENGTH_SHIN, 0) ;
    m.setBox(DENSITY, XSIDE, LENGTH_SHIN, ZSIDE);
    m.adjust(MASS_SHIN);
    body[BODY_RSHIN].setMass(&m);
    box[BODY_RSHIN].setBody(body[BODY_RSHIN]);
    // left foot
    body[BODY_LFOOT].setPosition(LENGTH_FOOT / 2, TORSO_POS - 0.5 * LENGTH_TORSO - LENGTH_THIGH - LENGTH_SHIN, 0) ;
    m.setBox(DENSITY, LENGTH_FOOT, XSIDE, ZSIDE);
    m.adjust(MASS_FOOT);
    body[BODY_LFOOT].setMass(&m);
    box[BODY_LFOOT].setBody(body[BODY_LFOOT]);
    // right foot
    body[BODY_RFOOT].setPosition(LENGTH_FOOT / 2, TORSO_POS - 0.5 * LENGTH_TORSO - LENGTH_THIGH - LENGTH_SHIN, 0) ;
    m.setBox(DENSITY, LENGTH_FOOT, XSIDE, ZSIDE);
    m.adjust(MASS_FOOT);
    body[BODY_RFOOT].setMass(&m);
    box[BODY_RFOOT].setBody(body[BODY_RFOOT]);

    // (left?) hip
    joint[JOINT_LHIP].attach(body[BODY_TORSO], body[BODY_LTHIGH]);
    joint[JOINT_LHIP].setAnchor(0, TORSO_POS - 0.5 * LENGTH_TORSO, 0);
    joint[JOINT_LHIP].setAxis(0, 0, 1.0);
    // right hip
    joint[JOINT_RHIP].attach(body[BODY_TORSO], body[BODY_RTHIGH]);
    joint[JOINT_RHIP].setAnchor(0, TORSO_POS - 0.5 * LENGTH_TORSO, 0);
    joint[JOINT_RHIP].setAxis(0, 0, 1.0);
    // left knee
    joint[JOINT_LKNEE].attach(body[BODY_LTHIGH], body[BODY_LSHIN]);
    joint[JOINT_LKNEE].setAnchor(0, TORSO_POS - 0.5 * LENGTH_TORSO - LENGTH_THIGH, 0);
    joint[JOINT_LKNEE].setAxis(0, 0, 1.0);
    // right knee
    joint[JOINT_RKNEE].attach(body[BODY_RTHIGH], body[BODY_RSHIN]);
    joint[JOINT_RKNEE].setAnchor(0, TORSO_POS - 0.5 * LENGTH_TORSO - LENGTH_THIGH, 0);
    joint[JOINT_RKNEE].setAxis(0, 0, 1.0);
    // left ankle
    joint[JOINT_LANKLE].attach(body[BODY_LSHIN], body[BODY_LFOOT]);
    joint[JOINT_LANKLE].setAnchor(0, TORSO_POS - 0.5 * LENGTH_TORSO - LENGTH_THIGH - LENGTH_SHIN, 0);
    joint[JOINT_LANKLE].setAxis(0, 0, 1.0);
    // right ankle
    joint[JOINT_RANKLE].attach(body[BODY_RSHIN], body[BODY_RFOOT]);
    joint[JOINT_RANKLE].setAnchor(0, TORSO_POS - 0.5 * LENGTH_TORSO - LENGTH_THIGH - LENGTH_SHIN, 0);
    joint[JOINT_RANKLE].setAxis(0, 0, 1.0);
}

ToyBody::ToyBody(Environment& env) : _env(env)
{
    for (int i = 0; i < NUM_BODY; i++)
    {
        body[i].create(_env.world);
    }
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        joint[i].create(_env.world);
    }
    box[BODY_TORSO].create(_env.space, XSIDE, LENGTH_TORSO, ZSIDE);
    box[BODY_LTHIGH].create(_env.space, XSIDE, LENGTH_THIGH, ZSIDE);
    box[BODY_RTHIGH].create(_env.space, XSIDE, LENGTH_THIGH, ZSIDE);
    box[BODY_LSHIN].create(_env.space, XSIDE, LENGTH_SHIN, ZSIDE);
    box[BODY_RSHIN].create(_env.space, XSIDE, LENGTH_SHIN, ZSIDE);
    box[BODY_LFOOT].create(_env.space, LENGTH_FOOT, XSIDE, ZSIDE);
    box[BODY_RFOOT].create(_env.space, LENGTH_FOOT, XSIDE, ZSIDE);

    for (int i = 0; i < NUM_BODY; i++)
    {
        joint_2d_cons[i] = dJointCreatePlane2D(_env.world.id(), 0);
        dJointAttach(joint_2d_cons[i], body[i].id(), 0);
    }

    positionBody();
}

void ToyBody::reset()
{
}

void ToyBody::render()
{
    dsSetTexture(DS_WOOD);
    for (int i = 0; i < NUM_BODY; i++)
    {
        dVector3 sides;
        int j = drawOrder[i];
        dGeomBoxGetLengths(box[j], sides);
        dBodyID bId = box[j].getBody();

        if (j == BODY_TORSO)
        {
            dsSetColor(1, 1, 0);
        }
        if (j == BODY_RTHIGH || j == BODY_RSHIN || j == BODY_RFOOT)
        {
            dsSetColor(0.3, 0.3, 0.6);
        }
        if (j == BODY_LTHIGH || j == BODY_LSHIN || j == BODY_LFOOT)
        {
            dsSetColor(0.3, 0.6, 0.3);
        }
        dsDrawBox(dBodyGetPosition(bId), dBodyGetRotation(bId), sides);
    }

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        dMatrix3 R;
        dRSetIdentity(R);
        dVector3 jpos, sides;
        sides[0] = sides[1] = sides[2] = 0.05;
        dsSetColor(0.0, 1.0, 1.0);
        dJointGetHingeAnchor(joint[i].id(), jpos);
        dsDrawBox(jpos, R, sides);
    }
}
