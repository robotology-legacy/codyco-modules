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
#include "math.h"
#include "iCub/linearRegressorsAdaptiveControl/linearRegressorsAdaptiveControlThread.h"


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::linearRegressorsAdaptiveControl;


linearRegressorsAdaptiveControlThread::linearRegressorsAdaptiveControlThread(ResourceFinder* _rf, string _robotName,
																			 wholeBodyInterface* _robot_interface,
																			 iDynTree * dynamical_model,
																			 const std::vector<bool> selected_DOFs,
																			 int period)
									   : rf(_rf), RateThread(period), robot_interface(_robot_interface), dynamical_model(_dynamical_model)
									   PERIOD(period), robotName(_robotName)
{
	N_DOFs = count_DOFs(selected_DOFs);

	q = dq = ddq = Vector(N_DOFs,0.0);
	q_d = dq_d = ddq_d = Vector(N_DOFs,0.0);
	dq_r = ddq_r = Vector(N_DOFs,0.0);
	s =	qTilde= Tau = Vector(N_DOFs,0.0);

	//N_p = what regressor you want to model?
	//Y = Matrix(N_DOFs,N_p);
	//Y.zero();
	//aHat = Vector(N_p,0.0);
	T_c = ((double)period)/1000.0;

	Kappa  = Vector(N_DOFs,0.0);
	Gamma  = Vector(N_p,0.0);
	Lambda = Vector(N_DOFs,0.0);
    
    //T_trajectory = ?
    trajectory_generator = minJerkTrajGen(N_DOFs,T_c,T_trajectory)
}

bool linearRegressorsAdaptiveControlThread::threadInit()
{
    //Read q_initial_complete 
    
    
    //Initialize the trajectory at the current state (so the it remain still)
    robot_interface->getQ(q.data());
    
    trajectory_generator.init(q);
}

void linearRegressorsAdaptiveControlThread::run()
{
    /* **************  READING SENSOR ******************************  */
    robot_interface->getQ(q.data());
    robot_interface->getDq(dq.data()); //to implement?
    
    /* **************  TRAJECTORY GENERATION ***********************  */
    qfPort.read(qf);
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
    trajectory_generator.computeNextValues(q_f);
    
    q_d = trajectory_generator.getPos();
    dq_d = trajectory_generator.getVel();
    ddq_d = trajectory_generator.getAcc();
    
	/* **************  VARIABLES TO COMPUTE CONTROL INPUTS ********************************************  */
    qTilde  = q     - q_d;                          /* Posititon error(s) */
    dqTilde = dq    - dq_d;                         /* Velocity error(s) */
    dq_r    = dq_d  - Lambda * qTilde;              /* Modified reference trajectories */
    ddq_r   = ddq_d - Lambda * dqTilde;
    s       = dq    - dqr;                          /* Modified position errors */
    
    //Yr      = Yr(q,dq,ddq_r);
    

    /* **************  CONTROL INPUTS *************************************************************  */
    Tau     = Yr * aHat - (norm(dq) + Kappa2) * Kappa * s;
    aHat    = aHat  - Gamma * ( Yr.transposed() * s ) * T_c;  /* Euler integration for estimated parameters  evolution */

    robot_interface->setPwmRef(Tau.data());
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

int linearRegressorsAdaptiveControlThread::
