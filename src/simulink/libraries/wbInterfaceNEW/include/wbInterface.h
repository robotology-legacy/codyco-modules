// Notes for new Simulink wbInterface

// 1. Change macro definitions from _ROBOT_STATE_CPP_ to _WBINTERFACE_
// 2. Methods and dependencies for the following two classes:
// 	a. robotStatus class (which should also change name to something more appropriate)
// 	b. counterClass
// 3. Remove all use namespace from header file.

// -----------------------------------------------------------------------------------

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

#ifndef _WBINTERFACE_H_
#define _WBINTERFACE_H_

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  robotState

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/Time.h>

#include <stdio.h>				// For printing debugging messages
#include <string.h>
#include <iostream>

#include <Eigen/Core>                           // import most common Eigen types
#include <Eigen/SVD>
#include <wbi/wbi.h>
#include <wbiIcub/wholeBodyInterfaceIcub.h>

#include "simstruc.h"				// Need to include simstruc.h for the definition of the SimStruct and its associated macro definitions.

typedef Eigen::Matrix<double,7,1>  Vector7d;
const int Dynamic = -1;
typedef Eigen::Matrix<double,6,Dynamic,Eigen::RowMajor>   JacobianMatrix;     // a Jacobian is 6 rows and N columns

static const Vector7d       	   DEFAULT_XDES_FOOT = Vector7d::Constant(0.0);
static const Eigen::Vector2d       DEFAULT_XDES_COM  = Eigen::Vector2d::Constant(0.0);
static const int      			   ICUB_DOFS = 25;    //This should somehow be provided by the user.

// ?????? DO I REALLY NEED THESE VARIABLES TO BE GLOBAL?? FIN A WAY TO REMOVE THEM!!! IT'S AWFUL AND DANGEROUS!!!!!!!!!
Eigen::VectorXd dotq;
yarp::sig::Vector qrad, xpose;
JacobianMatrix jacob;


class robotStatus {
private:
    // This object is used to control reentrancy
    static int 			creationCounter;
    int             		comLinkId;              // id of the COM
    int             		footLinkId;             // id of the controlled (swinging) foot link
    int 			actJnts;
    int                 	_n;                     // current number of active joints
    std::string 		moduleName;
    std::string 		robotName;
    Eigen::Map<Eigen::VectorXd>	dqDesMap;
    Eigen::VectorXd        	dq, dqJ;                // joint velocities (size of vectors: n+6, n, 6)
    Eigen::Matrix4d		H_w2b;                  // rotation matrix from world to base reference frame
    Eigen::VectorXd         	dqDes;
    yarp::sig::Vector        	x_pose;
    yarp::sig::Vector 		qRad;
    JacobianMatrix  		JfootR;                 // Jacobian of the right foot
    wbi::Frame           	Ha;                     // rotation to align foot Z axis with gravity, Ha=[0 0 1 0; 0 -1 0 0; 1 0 0 0; 0 0 0 1]
    wbi::Frame           	H_base_leftFoot;        // rototranslation from robot base to left foot (i.e. world)
    wbi::Frame 			xBase;


public:
    wbi::wholeBodyInterface *wbInterface;
    // Temporal container to copy wbInterface object to other copies of this module
    static int 		    *tmpContainer;
    
    robotStatus();
    ~robotStatus();
    void setmoduleName(std::string mn);
    void setRobotName(std::string rn); //checked
    int getCounter();
    int decreaseCounter();
    bool robotConfig();
    bool robotInit(int btype, int link);
    void getLinkId(const char *linkName, int &lid);
    //This is especifically for the COM
    int getLinkId(const char *linkName);
    bool world2baseRototranslation();
    bool robotJntAngles(bool blockingRead);
    bool robotJntVelocities(bool blockingRead);
    yarp::sig::Vector forwardKinematics(int &linkId);
    JacobianMatrix jacobian(int &lid);
    yarp::sig::Vector getEncoders();
    Eigen::VectorXd getJntVelocities();
    bool setCtrlMode(wbi::ControlMode ctrl_mode);
    void setdqDes(yarp::sig::Vector dqD);
    bool dynamicsMassMatrix(double *massMatrix);
    double dynamicsGenBiasForces(double *dxB, double *hterm);
};

// The initialization of this varibale must be done here because it's a pointer to static
// int robotStatus::creationCounter = 0;
// int *robotStatus::tmpContainer = NULL;

class counterClass {
private:
    static int count;
public:
    counterClass();
    int getCount();
};

// The initialization of this varibale must be done here because it's a pointer to static
// int counterClass::count = 0;

#endif