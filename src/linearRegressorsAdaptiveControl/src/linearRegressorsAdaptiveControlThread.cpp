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
																			 wholeBodyInterface* _robot, 
																			 iDynTree * dynamical_model, 
																			 const std::vector<bool> selected_DOFs,
																			 int period)
									   : rf(_rf), RateThread(period), robot(_robot),
									   PERIOD(period), robotName(_robotName)
{
	N_DOFs = count_DOFs(selected_DOFs);
	
	q = dq = ddq = Vector(N_DOFs,0.0);
	q_d = dq_d = ddq_d = Vector(N_DOFs,0.0);
	dq_r = ddq_r = Vector(N_DOFs,0.0);
	s =	q_err = Tau = Vector(N_DOFs,0.0);
	
	//N_p = what regressor you want to model?
	//Y = Matrix(N_DOFs,N_p);
	//Y.zero(); 
	//aH = Vector(N_p,0.0); 
	T_c = ((double)period)/1000.0;
	
	Kappa = Matrix(N_DOFs,N_DOFs);
	Gamma = Matrix(N_DOFs,N_DOFs);
	Lambda = Matrix(N_DOFs,N_DOFs);
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

bool linearRegressorsAdaptiveControlThread::threadInit()
{
	
}

void linearRegressorsAdaptiveControlThread::run(){

}


void linearRegressorsAdaptiveControlThread::threadRelease()
{
}
