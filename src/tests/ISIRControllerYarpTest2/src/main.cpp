
/*
 * Copyright (C) 2013 CODYCO Project
 * Author: Serena Ivaldi, Joseph Salini 
 * email:  serena.ivaldi@isir.upmc.fr, joseph.salini@isir.upmc.fr
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

/**
 
 A simple test of integration between ORCISIR and YARP.
  
 \section tested_os_sec Tested OS
 Linux
 
 \author Serena Ivaldi, Joseph Salini
 */

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <modHelp/modHelp.h>


#include <iostream>
#include <Eigen/Eigen>
// the model used to test ORCISIR
#include "iCubModel.h"
// these are to include ORCISIR
#include "orcisir/ISIRController.h"
#include "orcisir/Tasks/ISIREasyTaskManager.h" 

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace modHelp;
using namespace std;

// necessary for cartesian interfaces
YARP_DECLARE_DEVICES(icubmod)

int main(int argc, char** argv)
{
    YARP_REGISTER_DEVICES(icubmod)

    // first YARP
    Network yarp;
    if (!yarp.checkNetwork())
    {
        cout<<"YARP network not available. Aborting."<<endl;
        return -1;
    }

    // stuff related to connection with iCubSim
    string robot="icubSim";
    string name="test_orcisir";
    bool using_cartesian_arm_right=true;
    //
    PolyDriver *ddRA, *ddCartRA; 
    Property    optionsRA, optionsCartRA;
    //
    IPositionControl *iposRA;
    IEncoders *iencRA;
    ICartesianControl *icrtRA;
    //
    int startup_context_id_RA;
    //
    Vector xRA_des(3,0.0);
    Vector xRA(3,0.0), oRA(4,0.0);

    // right arm
    optionsRA.put("device","remote_controlboard");
    optionsRA.put("local",string("/"+name+"/right_arm").c_str());
    optionsRA.put("remote",string("/"+robot+"/right_arm").c_str());
    //
    optionsCartRA.put("device","cartesiancontrollerclient");
    optionsCartRA.put("remote",string("/"+robot+"/cartesianController/right_arm").c_str());
    optionsCartRA.put("local",string("/"+name+"/right_arm/cartesian").c_str());

    // right arm
    //....................................................
    if(!createDriver(ddRA,optionsRA))
    {
        cout<<"Problems connecting to the remote driver of right_arm"<<endl;
        return -1;
    }
    // cartesian interface
    if(using_cartesian_arm_right)
    {
        if(!ddRA->view(iencRA) || !ddRA->view(iposRA) )
        {
            cout<<"Problems acquiring interfaces of right_arm"<<endl;
            return -1;
        }

        if (!createDriver(ddCartRA,optionsCartRA))
        {
            cout<<"Problems connecting to the Cartesian interface of right_arm"<<endl;
            return -1;
        }
        if(!ddCartRA->isValid())
        {
            cout<<"Invalid Cartesian interface for right_arm"<<endl;
            return -1;
        }
        if(!ddCartRA->view(icrtRA))
        {
            cout<<"Problems acquiring the Cartesian interface of right_arm"<<endl;
            return -1;
        }

        cout<<"Latch controllers context for right arm"<<endl;
        icrtRA->storeContext(&startup_context_id_RA);

        icrtRA->setTrajTime(1.0);
    }

    


    //---------------------------------------------------------------------------------
    // now stuff related to the initialization of ORCISIR and its model

    // in iCubModel we have N= joints 
    VectorXd q      = Eigen::VectorXd(3);  
    VectorXd dq     = Eigen::VectorXd(3);
    VectorXd tau    = Eigen::VectorXd(3);
    double dt = 0.01;

    //SET CONTROLLER PARAMETERS
    cout<<"SET PARAMETERS\n";
    string internalSolver("qld");  //or "quadprog" "qld"
    bool useReducedProblem = false;     //true does not work yet...
    bool useMultiLevel     = false;

    // INITIALIZE ISIR MODEL, CONTROLLER & TASK MANAGER
    cout<<"INITIALIZE ISIR MODEL, CONTROLLER & TASK MANAGER\n";
    iCubModel                         model("iCubModel");  
    orcisir::ISIRController         ctrl("myCtrl", model, internalSolver, useReducedProblem, useMultiLevel);
    orcisir::ISIREasyTaskManager    taskManager(ctrl, model);


    //CREATE SOME TASKS
    cout<<"CREATE SOME TASKS\n";
    int refTaskIdx      = taskManager.createFullStateTask("refTask",       0, 0.0001, "INTERNAL");
    taskManager.setTaskStiffnessAndDamping(refTaskIdx,   9, 6);    //set Kp and Kd

    int frameTaskIdx    = taskManager.createPositionFrameTask("frameTask", 0, 1,  "Model3T.segment_3", Displacementd(), "XYZ");
    taskManager.setTaskStiffnessAndDamping(frameTaskIdx, 9, 6);
    taskManager.updateFrameTask(frameTaskIdx, Displacementd(-0.3, 0.3, 0.2), Twistd(0,0,0,0,0,0), Twistd());


    //---------------------------------------------------------------------------------
    // now simulation with ORCISIR and its model and connection with iCubSim

    //SIMULATE
    cout<<"SIMULATE\n";
    for (int i=0; i<1000; i++)
    {
        Time::delay(dt);

        cout<<"- -  --- - - - -- - - -- - "<<i<<"\n";
        ctrl.computeOutput(tau);    //compute tau
        cout<<"tau: "<<tau.transpose()<<"\n";

        VectorXd ddq = model.getAccelerationVariable().getValue();
        cout<<"ddq: "<<ddq.transpose()<<"\n";
        //VectorXd ddq = model.getInertiaMatrixInverse() * ( tau - model.getNonLinearTerms() - model.getGravityTerms() );

        dq += ddq * dt;
        q  += dq  * dt;

        model.setJointPositions(q);
        model.setJointVelocities(dq);
        // now send to icubsim
        // init icubsim right hand with the initial point of the model
        eigenToYarpVector(q, xRA_des);
        icrtRA->goToPosition(xRA_des);

        cout<<"pos seg3: "<< model.getSegmentPosition(3).getTranslation().transpose()<<"\n";

    }

    // end of simulation
    q = model.getJointPositions();
    icrtRA->getPose(xRA,oRA);

    cout<<"FINAL RESULT"<<endl
        <<"model   = "<<q<<endl
        <<"icubsim = "<<xRA.toString()<<endl;


    //---------------------------------------------------------------------------------
    // closing iCubSim and YARP stuff

    cout<<"Closing drivers"<<endl;

    if(using_cartesian_arm_right)
    {
       icrtRA->stopControl(); 
       icrtRA->restoreContext(startup_context_id_RA);
       deleteDriver(ddCartRA);
    }
    deleteDriver(ddRA);

    cout<<"ISIRControllerYARPTest is over"<<endl;

    return 0;
}
