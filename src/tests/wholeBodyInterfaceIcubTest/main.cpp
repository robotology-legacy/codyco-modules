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
#include <cmath>
#include <string>

#include <iostream>
#include <typeinfo>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::skinDynLib;
using namespace std;
using namespace wbi;
using namespace yarpWbi;
using namespace Eigen;

const double TOL = 1e-8;

const double WBITEST_RAD2DEG = 180.0 / M_PI;
const double WBITEST_DEG2RAD = M_PI / 180.0;


int main(int argc, char * argv[])
{
    Network yarpNet; 
    
  printf("Robot name is given \n\n");  
  string robotName = "icubGazeboSim";
  yarp::os::ResourceFinder rf;
  yarp::os::Property yarpWbiOptions;
  //Get wbi options from the canonical file
  
  int testType = 1;
  
  
  Property options;
  options.fromCommand(argc,argv);
  
  if( !rf.check("yarp") )
  {
      fprintf(stderr,"[ERR] locomotionControl: impossible to open wholeBodyInterface: wbi_conf_file option missing");
  }
  
  rf.setVerbose (true);
  rf.setDefaultConfigFile ("yarpWholeBodyInterface.ini");
  
  rf.configure(0,0);
  
  std::string wbiConfFile = rf.findFile("yarpWholeBodyInterface.ini");
  yarpWbiOptions.fromConfigFile(wbiConfFile);
  //Overwrite the robot parameter that could be present in wbi_conf_file
  yarpWbiOptions.put("robot",robotName);
    
  // TEST WHOLE BODY INTERFACE
  std::string localName = "wbiTest";
  wholeBodyInterface *icub;
 
    
#ifdef CODYCO_USES_URDFDOM
    icub =  new yarpWbi::yarpWholeBodyInterface(localName.c_str(), yarpWbiOptions);
#else
    //  iCub::iDynTree::iCubTree_version_tag icub_version;   
    //icub_version = iCub::iDynTree::iCubTree_version_tag (2, 2, true);
    //icub = new icubWholeBodyInterface (localName.c_str(), robotName.c_str(), icub_version, urdf_file);
    //icub =  new yarpWbi::yarpWholeBodyInterface(localName.c_str(), yarpWbiOptions);
#endif
    
    std::cout << "icubWholeBodyInterface created, adding joints" << std::endl;
    
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



  if(options.check("test")) {
        testType = options.find("test").asInt();
	std::cout<<"Test type selected : "<<testType;
  }

    
    Time::delay(0.5);
    
    int dof = icub->getDoFs();
    printf("Joint list: %s\n", icub->getJointList().toString().c_str());
    printf("Number of DoFs: %d\n", dof);
    
    Vector q(dof), dq(dof), d2q(dof), qInit(dof), qd(dof),basePos(16),basePosFromFrame(16);
    
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
   Vector refSpeed(dof, WBITEST_DEG2RAD*10.0);//, qd = q;
   
   if(testType == 1)
   {
    for (int ctr = 0; ctr <13;ctr++)
    {
      qd(ctr) += 15.0*WBITEST_DEG2RAD;
    }
   }
   else
   {
     //int keyJointsPlus[] = {13,17,19,23};
     int keyJointsPlus[] = {13,19,17,23};
     int keyJointsMinus[] = {16,22};
     
     for (int ctr = 0; ctr <2;ctr++)
     {
    qd(keyJointsPlus[ctr]) -= 15.0*WBITEST_DEG2RAD;
     }
     for (int ctr = 2; ctr <4;ctr++)
     {
    qd(keyJointsPlus[ctr]) += 15.0*WBITEST_DEG2RAD;
     }
     for (int ctr = 0; ctr <2;ctr++)
     {
    qd(keyJointsMinus[ctr]) -= 30.0*WBITEST_DEG2RAD;
     }
   }
   printf("Q:   %s\n", (WBITEST_RAD2DEG*q).toString(1).c_str());
   printf("Qd:  %s\n", (WBITEST_RAD2DEG*qd).toString(1).c_str());
   icub->setControlParam(CTRL_PARAM_REF_VEL, refSpeed.data());
   icub->setControlReference(qd.data());
   int j = 0;
   Eigen::Matrix<double,6,Dynamic,RowMajor> jacob; 
   jacob.resize(6,dof+6); //13 because in this test we only have right and left arm plus torso

   wbi::Frame basePosFrame;
   
   for(int i=0; i<15; i++)
   {
       Vector com(7,0.0);
       wbi::Frame world2base;
       world2base.identity();
       
       Time::delay(1);
       icub->getEstimates(ESTIMATE_JOINT_POS, q.data());
       icub->getEstimates(ESTIMATE_JOINT_VEL, dq.data());
       icub->getEstimates(ESTIMATE_JOINT_ACC,d2q.data());
        printf("(Q, dq, d2q):   %.2f \t %.2f \t %.2f\n", WBITEST_RAD2DEG*q(j), WBITEST_RAD2DEG*dq(j), WBITEST_RAD2DEG*d2q(j));
       
	icub->getEstimates(ESTIMATE_BASE_POS,basePos.data());
	printf("BasePos: %2.2f %2.2f %2.2f\n\n",basePos(3),basePos(7),basePos(11));
	printf("BaseRot: %2.2f %2.2f %2.2f\n %2.2f %2.2f %2.2f\n %2.2f %2.2f %2.2f\n\n\n",basePos(0),basePos(1),basePos(2),basePos(4),basePos(5),basePos(6),basePos(8),basePos(9),basePos(10));
               
	wbi::frameFromSerialization(basePos.data(),basePosFrame);
	wbi::serializationFromFrame(basePosFrame,basePosFromFrame.data());  
     
	printf("BasePos: %2.2f %2.2f %2.2f\n\n",basePosFromFrame(3),basePosFromFrame(7),basePosFromFrame(11));
	printf("BaseRot: %2.2f %2.2f %2.2f\n %2.2f %2.2f %2.2f\n %2.2f %2.2f %2.2f\n\n\n",basePosFromFrame(0),basePosFromFrame(1),basePosFromFrame(2),basePosFromFrame(4),basePosFromFrame(5),basePosFromFrame(6),basePosFromFrame(8),basePosFromFrame(9),basePosFromFrame(10));

  }

   printf("Test finished. Press return to exit.");
   getchar();
   
   printf("Q:   %s\n", (WBITEST_RAD2DEG*q).toString(1).c_str());

   qd -= WBITEST_DEG2RAD*15.0;
   icub->setControlMode(CTRL_MODE_POS);
   icub->setControlReference(qd.data());

   Time::delay(1.0);
   printf("Test finished. Press return to exit.");
   getchar();
   
   Vector refSpeedFinal(dof, WBITEST_DEG2RAD*25.0);//, qd = q;
//   qd += 15.0*WBITEST_DEG2RAD;
//   printf("Q:   %s\n", (WBITEST_RAD2DEG*q).toString(1).c_str());
//   printf("Qd:  %s\n", (WBITEST_RAD2DEG*qd).toString(1).c_str());
   icub->setControlParam(CTRL_PARAM_REF_VEL, refSpeedFinal.data());
   
   icub->setControlReference(qInit.data());
    
    printf("Test finished. Press return to exit.");
    getchar();
    
    icub->close();
    
    delete icub;
    
    printf("Main returning...\n");
    return 0;
}

