/*
 * Copyright (C) 2013 Italian Institute of Technology CoDyCo Project
 * Authors: Daniele Pucci and Silvio Traversaro
 * email:   daniele.pucci@iit.it and silvio.traversaro@iit.it
 * website: www.codyco.eu
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

#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/math/api.h>
#include "math.h"
#include "iCub/linearRegressorsAdaptiveControl/linearRegressorsAdaptiveControlThread.h"


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::linearRegressorsAdaptiveControl;


linearRegressorsAdaptiveControlThread::linearRegressorsAdaptiveControlThread(ResourceFinder* _rf, string _robotName,
																			 wholeBodyInterface* _robot_interface,
																			 DynTree * _dynamical_model,
																			 const std::vector<bool> _selected_DOFs,
																			 int period)
									   : rf(_rf), RateThread(period), robot_interface(_robot_interface), dynamical_model(_dynamical_model),
									   PERIOD(period), robotName(_robotName), selected_DOFs(_selected_DOFs)
{
	N_DOFs = count_DOFs(selected_DOFs);
    
    q_complete = dq_complete = ddq_r_complete = Vector(dynamical_model->getNrOfDOFs(),0.0);


	q = dq = Vector(N_DOFs,0.0);
	q_d = dq_d = ddq_d = Vector(N_DOFs,0.0);
	dq_r = ddq_r = Vector(N_DOFs,0.0);
	s =	qTilde= dqTilde = Tau = Vector(N_DOFs,0.0);


	N_p = getNrOfAdaptedParameters();
	Yr = Matrix(N_DOFs,N_p);
	Yr.zero();
	aHat = Vector(N_p,0.0);
	T_c = ((double)period)/1000.0;

	Kappa  = Vector(N_DOFs,0.0);
	Gamma  = Vector(N_p,0.0);
	Lambda = Vector(N_DOFs,0.0);
    
    friction_vec = Vector(4,0.0);
    
    //T_trajectory = ?
    T_trajectory = 1.0;
    trajectory_generator = new minJerkTrajGen(N_DOFs,T_c,T_trajectory);
}

bool linearRegressorsAdaptiveControlThread::threadInit()
{
    //Initialize the trajectory at the current state (so the it remain still)
    robot_interface->getQ(q_complete.data());
    
    selectActiveDOFs(q_complete,q);
    
    trajectory_generator->init(q);
    
    Vector inertial_parameters;
    dynamical_model->getDynamicsParameters(inertial_parameters);
    aHat.setSubvector(0,inertial_parameters);
}

void linearRegressorsAdaptiveControlThread::run()
{
    /* **************  READING SENSOR ******************************  */
    robot_interface->getQ(q_complete.data());
    robot_interface->getDq(dq_complete.data()); //to implement?
    
    selectActiveDOFs(q_complete,q);
    selectActiveDOFs(dq_complete,dq);
    
    /* **************  TRAJECTORY GENERATION ***********************  */
    Vector * tmp = qfPort.read();
    qf = *tmp;
    
    if( qf.size() != N_DOFs ) {
        //abort
        YARP_ASSERT(false);
    }
    
    {
        //Check limits
        /** Todo: add limits support to 
         *  iDynTree or wholeBodyInterface
         * 
         */
        YARP_ASSERT(false);
    }
    
    //Generate trajectory from qf and obtain q_d, dq_d
    trajectory_generator->computeNextValues(qf);
    
    q_d = trajectory_generator->getPos();
    dq_d = trajectory_generator->getVel();
    ddq_d = trajectory_generator->getAcc();
    
	/* **************  VARIABLES TO COMPUTE CONTROL INPUTS ********************************************  */
    qTilde  = q     - q_d;                          /* Posititon error(s) */
    dqTilde = dq    - dq_d;                         /* Velocity error(s) */
    dq_r    = dq_d  - Lambda * qTilde;              /* Modified reference trajectories */
    ddq_r   = ddq_d - Lambda * dqTilde;
    s       = dq    - dq_r;                          /* Modified position errors */
    
    setActiveDOFs(ddq_r,ddq_r_complete);
    computeRegressor();

    /* **************  CONTROL INPUTS *************************************************************  */
    Tau     = Yr * aHat - (norm(dq) + Kappa2) * Kappa * s;
    aHat    = aHat  - Gamma * ( Yr.transposed() * s ) * T_c;  /* Euler integration for estimated parameters  evolution */

    int reduced_i = 0;
    for(int i=0; i < selected_DOFs.size(); i++ ) {
        if( selected_DOFs[i] ) { 
            robot_interface->setTorqueRef(&(Tau[reduced_i]),i);
            reduced_i++;
        }
    }
}


void linearRegressorsAdaptiveControlThread::threadRelease()
{
}


int linearRegressorsAdaptiveControlThread::count_DOFs(const std::vector<bool> & selected_DOFs)
{
	int DOFs = 0;
	for(int i = 0; i < selected_DOFs.size(); i++ ) {
		if( selected_DOFs[i] ) {
			DOFs++;
		}
	}
	return DOFs;
}

void linearRegressorsAdaptiveControlThread::selectActiveDOFs(const Vector & vec_complete, Vector & vec)
{
    int reduced_i = 0;
    for(int i=0; i < selected_DOFs.size(); i++ ) 
    {
        if( selected_DOFs[i] ) {
            vec(reduced_i) = vec_complete(i);
            reduced_i++;
        }
    }
    return;
}

void linearRegressorsAdaptiveControlThread::setActiveDOFs(const Vector & vec, Vector & vec_complete)
{
    int reduced_i = 0;
    for(int i=0; i < (int)selected_DOFs.size(); i++ ) 
    {
        if( selected_DOFs[i] ) {
            vec_complete(i) = vec(reduced_i);
            reduced_i++;
        }
    }
    return;
}

int linearRegressorsAdaptiveControlThread::getNrOfAdaptedParameters()
{
    return 10*(dynamical_model->getNrOfLinks())+4*N_DOFs;
}


double positive_part(double dq)
{
    if( dq > 0.0 ) return dq;
    return 0.0;
}

double negative_part(double dq)
{
    if( dq < 0.0 ) return -dq;
    return 0.0;
}
     
double damped_sgn(double dq, double tol)
{
    tol = fabs(tol);
    if( dq < tol && dq > -tol ) {
        return 0.0;
    }
    if( dq > tol ) {
        return 1.0;
    }
    if( dq < -tol ) {
        return -1.0;
    }
    return 0.0;
}

void friction_regressor(const double dq, Vector & regr)
{
    double tol = CTRL_DEG2RAD*1;
    regr(0) = positive_part(damped_sgn(dq,tol));
    regr(1) = -negative_part(damped_sgn(dq,tol));
    regr(2) = positive_part(dq);
    regr(3) = -negative_part(dq);
    return;
}


void linearRegressorsAdaptiveControlThread::computeRegressor()
{
    dynamical_model->setAng(q_complete);
    dynamical_model->setDAng(dq_complete);
    dynamical_model->setD2Ang(ddq_r_complete);
    dynamical_model->getDynamicsRegressor(Y_complete_no_friction);
    
    int reduced_i = 0;
    for(int i = 0; i <  selected_DOFs.size(); i++ ) {
        if( selected_DOFs[i] ) {
            Yr.setSubrow(Y_complete_no_friction.getRow(i),reduced_i,0);
            reduced_i++;
        }
    }
    
    for(reduced_i=0; reduced_i < N_DOFs; reduced_i++ ) {
        double dq_reduced_i = dq[reduced_i];
        friction_regressor(dq_reduced_i,friction_vec);
        Yr.setSubrow(friction_vec,reduced_i,10*(dynamical_model->getNrOfLinks())+4*reduced_i);
    }
    
    return;
}

bool linearRegressorsAdaptiveControlThread::setGain(available_gains gain, double value)
{
    if( value <= 0.0 ) return false;
    switch( gain ) {
        case gamma_gain:
            Gamma = value;
            return true;
        case kappa_gain:
            Kappa = value;
            return true;
        case lambda_gain:
            Lambda = value;
            return true;
        case trajectory_time:
            T_trajectory = value;
    }
}
