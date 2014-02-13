/*
 * Copyright (C) 2013 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Jorhabib Eljaik Gomez
 * email: jorhabib.eljaik@iit.it
 *
 * The development of this software was supported by the FP7 EU project
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 * http://www.codyco.eu
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef _MIN_JERK_H_
#define _MIN_JERK_H_

#include <yarp/os/all.h>
#include <yarp/os/Network.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>

#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/minJerkCtrl.h>

#include <iostream>
#include <stdio.h>

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;

using namespace iCub::ctrl;
using namespace std;


// All these static consts could be put in an external file
static const int type = 1;
static const int DEFAULT_COM_TASK   = 2;
static const int DEFAULT_FOOT_TASK  = 7;
static const int ICUB_DOFS = 25;
static const int DEFAULT_POSE_TASK = ICUB_DOFS;

//static const Vector2d       DEFAULT_XDES_COM        = Vector2d::Constant(0.0);
//static const Vector7d       DEFAULT_XDES_FOOT       = Vector7d::Constant(0.0);
//static const VectorNd       DEFAULT_QDES            = VectorNd::Constant(0.0);
//static const Matrix4d       DEFAULT_H_W2B           = Matrix4d::Identity();
//static const Vector2d       DEFAULT_XREF_COM        = Vector2d::Constant(0.0);
//static const Vector7d       DEFAULT_XREF_FOOT       = Vector7d::Constant(0.0);
//static const VectorNd       DEFAULT_QREF            = VectorNd::Constant(0.0);
//static const Vector2d       DEFAULT_X_COM           = Vector2d::Constant(0.0);
//static const Vector7d       DEFAULT_X_FOOT          = Vector7d::Constant(0.0);
//static const VectorNd       DEFAULT_Q               = VectorNd::Constant(0.0);

class minJerkTrajGenerator{
private:
    minJerkTrajGen *trajGen;
    Vector ref, dref, ddref;
    double SIMU_RATE;
    //  This vector pos comes from the ROBOT STATE blocks and will be
    //  either x_com, x_foot or qRad (for robot pose) given by the forward
    //  kinematics blocks
    Vector pos, pos_init;
    //  The following method helps to conditionally instantiate
    //  trajGen depending on the kind of block this will be
    static int condTaskDOF(int n)
    {
        switch(n){
        case 0: //foot
            return DEFAULT_FOOT_TASK;
            break;
        case 1: //COM
            return DEFAULT_COM_TASK;
            break;
        case 2: //POSE
            return DEFAULT_POSE_TASK;
            break;
        }
    }
    int taskDOF;

public:
    minJerkTrajGenerator(int tdof){
        trajGen = NULL;
        taskDOF = tdof;     //Dimension of the minimun jerk trajectory necessary for initialization of pos_init which must be done before minJerkTrajGenerator::initialize so that it can be set with the input to the block in mdlStart
        pos_init.resize(taskDOF, 0.0);
    }
    ~minJerkTrajGenerator(){
        if(trajGen){
            delete trajGen;
            trajGen = NULL;
        }
    }

    bool initialize(int RATE, double TT){

        //        TDOF = condTaskDOF(type);     //This hould be uncommented if you wanna choose from three particular tasks as described in the switch above
        SIMU_RATE = static_cast<double>(RATE)/1000;
        pos.resize      (taskDOF  ,1.0);
        ref.resize      (taskDOF  ,0.0);  // Result of getPos(). Reference trajectory vector resizing.
        dref.resize     (taskDOF  ,0.0);  // Result of getVel(). Reference trajectory velocity vector resizing.
        ddref.resize    (taskDOF  ,0.0);  // Result of getAcc().

        trajGen = new minJerkTrajGen(taskDOF, SIMU_RATE, TT);
        trajGen->init(pos_init);
        return true;
    }

    void updateReferenceTraj(){
//        fprintf(stderr,"computeNextValues take pos with value: %s \n",pos.toString().c_str());
        trajGen->computeNextValues(pos);
//        fprintf(stderr,"pos: %s \n",pos.toString().c_str());
        return;
    }

    //The following two methods should be made ONE.
    void setInitPos(int idx, double val){
        pos_init(idx) = val;
        return;
    }
    void setPos(int idx, double val){
        pos(idx) = val;
        return;
    }

    Vector getInitPos(){
        return pos_init;
    }

    Vector getPosition(){
        ref = trajGen->getPos();
//        fprintf(stderr,"ref position: %s \n", ref.toString().c_str());
        return ref;
    }
    Vector getVelocity(){
        dref = trajGen->getVel();
//        fprintf(stderr,"ref velocity: %s \n", dref.toString().c_str());
        return dref;
    }
    Vector getAcceleration(){
        ddref = trajGen->getAcc();
//        fprintf(stderr,"ref Acceleration: %s \n", ddref.toString().c_str());
        return ddref;
    }
};


#endif


