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

#include <iostream>
#include <string>
#include <vector>

#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/os/Network.h>	//temporary

#include <iCub/ctrl/minJerkCtrl.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace wbi;

namespace iCub{

namespace linearRegressorsAdaptiveControl{

class linearRegressorsAdaptiveControlThread : public RateThread
{
public:
	/* class methods */
	linearRegressorsAdaptiveControlThread(ResourceFinder* rf,
										  string robotName,
										  wholeBodyInterface* robot_interface,
										  iDynTree* dynamical_model,
										  const std::vector<bool> selected_DOFs,
										  int period);
	bool threadInit();
	void threadRelease();
	void run();

private:

	/* class constants */
	const int PERIOD;

	/* class variables */
    wholeBodyInterface* robot_interface;
    
    iDynTree* dynamical_model;
    
	ResourceFinder* rf;

	// input parameters
	string robotName;

	/* ports */
	BufferedPort<Vector> qfPort;
    

    /* Mathematical variables */    
    Vector q_complete, dq_complete; /**< Complete state variables */
    
    Vector ddq_r_complete; 
    
    int N_DOFs; /**< adaptivly controlled Degrees of Freedom */

    int N_p; /**< Number of parameters */
    

    Vector q, dq; /**< State variables */

	Vector q_d, dq_d, ddq_d; /**< Reference trajectories */

	Vector dq_r, ddq_r; /**< Modified reference variables */
    

	Vector s; /**< Modified error variable (dq-dq_r) */

	Vector qTilde; /**< Position error (q-q_d) */

	Vector Tau; /**< Torques */

	Vector aHat; /**< Estimated parameters */

	Matrix Yr; /**< Regressor matrix */
    
    Matrix Y_complete_no_friction; /**< Regressor matrix with all joints, but without friction */

	double T_c; /**< Timestamp in s */

	/* Gains */
	Vector Lambda, Gamma, Kappa; /** Gain vector */

    double Kappa2;
    
    //Bool vector of activly controlled DOF 
    std::vector<bool> selected_DOFs;
    
	//Helper methods
	int count_DOFs(const std::vector<bool> & selected_DOFs);
    
    //Dummy varibales
    Vector friction_vec;
    
    //Trajectory generation
    minJerkTrajGen trajectory_generator;
    double T_trajectory;

};

} //namespace iCub

} //namespace
