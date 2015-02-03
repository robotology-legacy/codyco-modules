/*
 * Author: Andrea Del Prete.
 * Copyright (C) 2013 The Robotcub consortium.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


/**
 * \infile Tests for wholeBodyInterfaceYarp.
 */
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Property.h>

#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Rand.h>

#include <iCub/skinDynLib/common.h>

//#include <wbiIcub/wholeBodyInterfaceIcub.h>
//#include <wbiIcub/icubWholeBodyModel.h>
#include<yarpWholeBodyInterface/yarpWholeBodyInterface.h>

#include <stdio.h>
#include <math.h>
#include <string>

#include <iostream>
#include <typeinfo>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::skinDynLib;
using namespace std;
using namespace wbi;
using namespace yarpWbi;//wbiIcub;
using namespace Eigen;

const double TOL = 1e-8;


int main(int argc, char * argv[])
{
    Network yarpNet; 
    /*
    Property options;
    options.fromCommand(argc,argv);
    
    std::string robotName;
    if(options.check("robot")) {
      robotName = options.find("robot").asString();
    } else {
      robotName = "icubGazeboSim";
    }
    
    bool use_urdf = false;
    
    std::string urdf_file;
    if(options.check("urdf")) { 
        use_urdf = true;
        urdf_file = options.find("urdf").asString();
    } else {
        use_urdf = false;
    }
    
    */
    
  printf("Robot name is given \n\n");  
  string robotName = "icubGazeboSim";
  yarp::os::ResourceFinder rf;
  yarp::os::Property yarpWbiOptions;
  //Get wbi options from the canonical file
  if( !rf.check("yarp") )
  {
      fprintf(stderr,"[ERR] locomotionControl: impossible to open wholeBodyInterface: wbi_conf_file option missing");
  }
  
  rf.setVerbose (true);
  rf.setDefaultConfigFile ("yarpWholeBodyInterface.ini");
//   rf.setDefaultContext ("icubGazeboSim");
  
  rf.configure(0,0);
  
  std::string wbiConfFile = rf.findFile("yarpWholeBodyInterface.ini");
  yarpWbiOptions.fromConfigFile(wbiConfFile);
  //Overwrite the robot parameter that could be present in wbi_conf_file
  yarpWbiOptions.put("robot",robotName);
    
    // TEST WHOLE BODY INTERFACE
    std::string localName = "wbiTest";
    
    wholeBodyInterface *icub;
 
    
#ifdef CODYCO_USES_URDFDOM
    //icub_version = iCub::iDynTree::iCubTree_version_tag (2, 2, true, true, urdf_file);
    //icub = new icubWholeBodyInterface (localName.c_str(), robotName.c_str(), icub_version);
    icub =  new yarpWbi::yarpWholeBodyInterface(localName.c_str(), yarpWbiOptions);
#else
    //  iCub::iDynTree::iCubTree_version_tag icub_version;   
    //icub_version = iCub::iDynTree::iCubTree_version_tag (2, 2, true);
    //icub = new icubWholeBodyInterface (localName.c_str(), robotName.c_str(), icub_version, urdf_file);
    //icub =  new yarpWbi::yarpWholeBodyInterface(localName.c_str(), yarpWbiOptions);
#endif
    
    
    
    std::cout << "icubWholeBodyInterface created, adding joints" << std::endl;
//    icub->addJoints(LocalIdList(RIGHT_ARM,0,1,2,3,4));
//    icub->addJoints(LocalIdList(LEFT_ARM,0,1,2,3,4));
//    icub->addJoints(LocalIdList(TORSO,0,1,2));
//    icub->addJoints(ICUB_MAIN_JOINTS);
    
  wbi::IDList RobotMainJoints;
  std::string RobotMainJointsListName = "ROBOT_TORQUE_CONTROL_JOINTS";
  if( !yarpWbi::loadIdListFromConfig(RobotMainJointsListName,yarpWbiOptions,RobotMainJoints) )
  {
      fprintf(stderr, "[ERR] locomotionControl: impossible to load wbiId joint list with name %s\n",RobotMainJointsListName.c_str());
  }	
  icub->addJoints(RobotMainJoints);
    
    //icub->addFTsens(LocalId(RIGHT_LEG,1));
    std::cout << "Joints added, calling init method" <<  std::endl;

    if(!icub->init())
        return 0;
    
    Time::delay(0.5);
    
    int dof = icub->getDoFs();
    printf("Joint list: %s\n", icub->getJointList().toString().c_str());
    printf("Number of DoFs: %d\n", dof);
    
    Vector q(dof), dq(dof), d2q(dof), qInit(dof), qd(dof),basePos(12);
    
    //Vector qinit(dof);
    double timeIni = Time::now();
    icub->getEstimates(wbi::ESTIMATE_JOINT_POS, q.data());
    double timeEnd = Time::now();
    
    //qinit = q;
    
    
    double elapsedTime = timeEnd - timeIni;
    printf("Elapsed time for ESTIMATE_JOINT_POS %f \n", elapsedTime);
    /*
    timeIni = Time::now();
    icub->getEstimates(wbi::ESTIMATE_JOINT_TORQUE, q.data());
    timeEnd = Time::now();
    
    elapsedTime = timeEnd - timeIni;
    printf("Elapsed time for ESTIMATE_JOINT_TORQUE %f \n", elapsedTime);
    */
    qInit = q;
    qd = q;
   Vector refSpeed(dof, CTRL_DEG2RAD*10.0);//, qd = q;
   
   for (int ctr = 0; ctr <13;ctr++)
   {
     qd(ctr) += 15.0*CTRL_DEG2RAD;
   }
   
   printf("Q:   %s\n", (CTRL_RAD2DEG*q).toString(1).c_str());
   printf("Qd:  %s\n", (CTRL_RAD2DEG*qd).toString(1).c_str());
   icub->setControlParam(CTRL_PARAM_REF_VEL, refSpeed.data());
   icub->setControlReference(qd.data());
   int j = 0;
   Eigen::Matrix<double,6,Dynamic,RowMajor> jacob; 
   jacob.resize(6,dof+6); //13 because in this test we only have right and left arm plus torso

   for(int i=0; i<15; i++)
   {
       Vector com(7,0.0);
       wbi::Frame world2base;
       world2base.identity();
       
       Time::delay(1);
       icub->getEstimates(ESTIMATE_JOINT_POS, q.data());
       icub->getEstimates(ESTIMATE_JOINT_VEL, dq.data());
       icub->getEstimates(ESTIMATE_JOINT_ACC,d2q.data());
        printf("(Q, dq, d2q):   %.2f \t %.2f \t %.2f\n", CTRL_RAD2DEG*q(j), CTRL_RAD2DEG*dq(j), CTRL_RAD2DEG*d2q(j));
       
	icub->getEstimates(ESTIMATE_BASE_POS,basePos.data());
	printf("BasePos: %2.2f %2.2f %2.2f\n\n",basePos(0),basePos(1),basePos(2));
	
	printf("BaseRot: %2.2f %2.2f %2.2f\n %2.2f %2.2f %2.2f\n %2.2f %2.2f %2.2f\n\n\n",basePos(3),basePos(4),basePos(5),basePos(6),basePos(7),basePos(8),basePos(9),basePos(10),basePos(11));
               
   }

   printf("Test finished. Press return to exit.");
   getchar();
   
   printf("Q:   %s\n", (CTRL_RAD2DEG*q).toString(1).c_str());

   qd -= CTRL_DEG2RAD*15.0;
   icub->setControlMode(CTRL_MODE_POS);
   icub->setControlReference(qd.data());

   Time::delay(1.0);
   printf("Test finished. Press return to exit.");
   getchar();
   
   Vector refSpeedFinal(dof, CTRL_DEG2RAD*25.0);//, qd = q;
//   qd += 15.0*CTRL_DEG2RAD;
//   printf("Q:   %s\n", (CTRL_RAD2DEG*q).toString(1).c_str());
//   printf("Qd:  %s\n", (CTRL_RAD2DEG*qd).toString(1).c_str());
   icub->setControlParam(CTRL_PARAM_REF_VEL, refSpeedFinal.data());
   
   icub->setControlReference(qInit.data());
    
    printf("Test finished. Press return to exit.");
    getchar();
    
    icub->close();
    
    delete icub;
    
    printf("Main returning...\n");
    return 0;
}

