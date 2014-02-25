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

// Need to include simstruc.h for the definition of the SimStruct and its associated macro definitions.
#include "simstruc.h"				     

//This should somehow be provided by the user, but 25 will be the default 
#define	ICUB_DOFS 25

typedef Eigen::Matrix<double,7,1>  Vector7d;
const int Dynamic = -1;
// a Jacobian is 6 rows and N columns
typedef Eigen::Matrix<double,6,Dynamic,Eigen::RowMajor>   	      JacobianMatrix;     
// N+6 x N+6 mass matrix
typedef Eigen::Matrix<double,ICUB_DOFS+6,ICUB_DOFS+6,Eigen::RowMajor> MassMatrix; 	  

static const Vector7d       	   DEFAULT_XDES_FOOT = Vector7d::Constant(0.0);
static const Eigen::Vector2d       DEFAULT_XDES_COM  = Eigen::Vector2d::Constant(0.0);


// ?????? DO I REALLY NEED THESE VARIABLES TO BE GLOBAL?? FIN A WAY TO REMOVE THEM!!! IT'S AWFUL AND DANGEROUS!!!!!!!!!
Eigen::VectorXd dotq;
yarp::sig::Vector qrad, xpose;
JacobianMatrix jacob;


class robotStatus {
private:
    // This object is used to control reentrancy.
    static int 			creationCounter;
    // id of the COM that is different from the other parts of the robot body.
    int             		comLinkId;              
    // id of the controlled foot link.
    int             		footLinkId;   
    // Current active joints (not being update at the moment 24/02/2014 12:44pm)
    int 			actJnts;
    // current number of active joints.
    int                 	_n;     
    // Prefix given to the ports that will be open by Simulink.
    std::string 		moduleName;
    // name of the robot being used, e.g. 'icubSim' or 'icub'.
    std::string 		robotName;
    // This variable map an Eigen vector to a yarp vector. 
    Eigen::Map<Eigen::VectorXd>	dqDesMap;
    // Joint velocities (size of vectors: n+6, n, 6)
    Eigen::VectorXd        	dq, dqJ;                
    // rotation matrix from world to base reference frame.
    Eigen::Matrix4d		H_w2b;                  
    Eigen::VectorXd         	dqDes;
    // General vector initialized depending on the link for which forwardKinematics is computed as well as dJdq.
    yarp::sig::Vector        	x_pose;
    // Robot joint angles in radians later initialized for the entire body.
    yarp::sig::Vector 		qRad;
    // General Jacobian matrix initialized depending on the link for which the Jacobian is then needed.
    JacobianMatrix  		JfootR;                 
     // rotation to align foot Z axis with gravity, Ha=[0 0 1 0; 0 -1 0 0; 1 0 0 0; 0 0 0 1]
    wbi::Frame           	Ha;                    
    // rototranslation from robot base to left foot (i.e. world).
    wbi::Frame           	H_base_leftFoot;   
    // Floating base 3D rototranslation from world ot base.
    wbi::Frame 			xBase;
    // Floating base velocity.
    yarp::sig::Vector		dxB;
    // Generalized bias forces.
    yarp::sig::Vector		hterm;
    yarp::sig::Vector    	dJdq;
    // Mass matrix
    MassMatrix			massMatrix;


public:
    wbi::wholeBodyInterface *wbInterface;
    // Temporal container to copy wbInterface object to other copies of this module
    static int 		    *tmpContainer;
    
    robotStatus();
    ~robotStatus();
    void 		setmoduleName(std::string mn);
    void 		setRobotName(std::string rn); //checked
    int 		getCounter();
    int 		decreaseCounter();
    bool 		robotConfig();
    bool 		robotInit(int btype, int link);
    void 		getLinkId(const char *linkName, int &lid);
    //This is especifically for the COM
    int 		getLinkId(const char *linkName);
    bool 		world2baseRototranslation();
    bool 		robotJntAngles(bool blockingRead);
    bool 		robotJntVelocities(bool blockingRead);
    yarp::sig::Vector 	forwardKinematics(int &linkId);
    JacobianMatrix 	jacobian(int &lid);
    yarp::sig::Vector 	getEncoders();
    Eigen::VectorXd 	getJntVelocities();
    bool 		setCtrlMode(wbi::ControlMode ctrl_mode);
    void      		setdqDes(yarp::sig::Vector dqD);
    bool       		dynamicsMassMatrix();
    
    yarp::sig::Vector	dynamicsGenBiasForces();
    bool       		robotBaseVelocity();
    bool       		dynamicsDJdq(int &linkId);
    MassMatrix 		getMassMatrix();
    yarp::sig::Vector 	getDJdq();
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