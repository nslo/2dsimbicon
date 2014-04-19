#ifndef _ENVIRONMENT_H_
#define _ENVIRONMENT_H_

#include <iostream>
#include <ode/ode.h>

const double GLB_CFM = 1e-10;
const double GLB_ERP = 0.8;

struct Environment
{
    dWorld world;
    dSimpleSpace space;
    dPlane ground;
    double gx, gy, gz;
    double gmag;
    double contact_cfm;
    double slope;

    Environment(double gx, double gy, double gz) :
        space(0), gx(gx), gy(gy), gz(gz), contact_cfm(GLB_CFM)
    {
        world.setGravity(gx, gy, gz);
        //dWorldSetCFM(world.id(), GLB_CFM);
        //dWorldSetERP(world.id(), GLB_ERP);
        ground.create(space, 0, 1, 0, 0);
        gmag = sqrt(gx * gx + gy * gy + gz * gz);
        slope = 0;
    }

    Environment(double magnitude, double degree) : space(0)
    {
        gmag = magnitude;
        gx = 0;
        gy = -magnitude * sin(degree * M_PI / 180.0);
        gz = -magnitude * cos(degree * M_PI / 180.0);
        world.setGravity(gx, gy, gz);
        dWorldSetCFM(world.id(), GLB_CFM);
        dWorldSetERP(world.id(), GLB_ERP);
        ground.create(space, 0, 1, 0, 0);
        contact_cfm = GLB_CFM;
        slope = 0;
    }

    void setSlope(double degree)
    {
        slope = degree;
        ground.setParams(sin(degree * M_PI / 180.0),
                cos(degree * M_PI / 180.0),
                0 , 0);
        dVector4 result;
        ground.getParams(result);
    }

    double getSlope()
    {
        return slope;
    }

    void setMagnitude(double mag)
    {
        gx = 0;
        gy = (mag / gmag) * gy;
        gz = (mag / gmag) * gz;
        gmag = mag;
        world.setGravity(gx, gy, gz);
    }

    void gravityOn()
    {
        world.setGravity(gx, gy, gz);
    }

    void gravityOff()
    {
        world.setGravity(0, 0, 0);
    }
};

#endif
