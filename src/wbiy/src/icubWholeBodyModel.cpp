/*
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email: marco.randazzo@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "wbiy/wbiy.h"
#include <string>

using namespace std;
using namespace wbi;
using namespace wbiy;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

#define MAX_NJ 20
#define WAIT_TIME 0.001

// iterate over all body parts
#define FOR_ALL_BODY_PARTS(itBp)            FOR_ALL_BODY_PARTS_OF(itBp, jointIdList)
// iterate over all joints of all body parts
#define FOR_ALL(itBp, itJ)                  FOR_ALL_OF(itBp, itJ, jointIdList)

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          ICUB WHOLE BODY MODEL
// *********************************************************************************************************************
// *********************************************************************************************************************
bool icubWholeBodyModel::init()
{
    return false;
}

int icubWholeBodyModel::getDoFs()
{
    return 0;
}

bool icubWholeBodyModel::removeJoint(const wbi::LocalId &j)
{
    return false;
}

bool icubWholeBodyModel::addJoint(const wbi::LocalId &j)
{
    return false;
}

int icubWholeBodyModel::addJoints(const wbi::LocalIdList &j)
{
    return false;
}

bool icubWholeBodyModel::getJointLimits(double *qMin, double *qMax, int joint)
{
    return false;    
}

bool icubWholeBodyModel::computeH(double *q, double *xBase, int linkId, double *H)
{
    return false;    
}

bool icubWholeBodyModel::computeJacobian(double *q, double *xBase, int linkId, double *J, double *pos)
{
    return false;    
}

bool icubWholeBodyModel::computeDJdq(double *q, double *xB, double *dq, double *dxB, int linkId, double *dJdq, double *pos)
{
    return false;    
}

bool icubWholeBodyModel::forwardKinematics(double *q, double *xB, int linkId, double *x)
{
    return false;
}

bool icubWholeBodyModel::inverseDynamics(double *q, double *xB, double *dq, double *dxB, double *ddq, double *ddxB, double *tau)
{
    return false;    
}

bool icubWholeBodyModel::directDynamics(double *q, double *xB, double *dq, double *dxB, double *M, double *h)
{
    return false;
}
