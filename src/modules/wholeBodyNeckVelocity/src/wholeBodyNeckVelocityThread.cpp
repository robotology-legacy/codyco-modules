#include "wholeBodyNeckVelocityThread.h"
#include <boost/concept_check.hpp>

WholeBodyNeckVelocityThread::WholeBodyNeckVelocityThread(wbi::wholeBodyInterface& robotInterface, int rate): RateThread(rate), m_robotInterface(robotInterface)
{
  lastRot.R = wbi::Rotation (0, 0, 1, 0, -1, 0, 1, 0, 0);  
}

WholeBodyNeckVelocityThread::~WholeBodyNeckVelocityThread()
{

}

void WholeBodyNeckVelocityThread::run()
{

  // Support foot TODO This must be updated every cycle according to F/T sensors input
  FOOT supportFoot = LEFT_FOOT;
  
  // Distance from previous base TODO This must be updated every cycle
  yarp::sig::Vector distanceToWRF(3);
  distanceToWRF.zero();
 
  
  // compute neck velocity
  yarp::sig::Vector neckVelocity(3);
  neckVelocity.zero();
  if(!computeNeckVelocity(neckVelocity.data(), supportFoot, distanceToWRF.data())) {
      std::cerr << "ERR computing neck velocity!" << std::endl;
      return;
  }
}

bool WholeBodyNeckVelocityThread::threadInit()
{
  return true;

}

void WholeBodyNeckVelocityThread::threadRelease()
{
yarp::os::RateThread::threadRelease();
}

bool WholeBodyNeckVelocityThread::computeNeckVelocity(double*                  neckVelocity, 
						      FOOT                     supportFoot, 
						      double*                  distanceToPreviousBase)
{
    wbi::Frame supportFootToRoot;
    computeSupportFootToRoot(supportFoot, supportFootToRoot);
    
    // get joint angles
    yarp::sig::Vector q_rad;
    q_rad.resize(m_robotInterface.getDoFs(), 0.0);
    m_robotInterface.getEstimates(wbi::ESTIMATE_JOINT_POS, q_rad.data());
  
    // compute jacobian
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::RowMajor> jacobianNeck;
    int neckID = -1;
    m_robotInterface.getFrameList().idToIndex("neck", neckID);
    
    // TODO Before passing supportFootToRoot to computeJacobian I need to know whether
    // it takes into account the fact that the WRF In this case is outside of the robot.
    // If it does, I need to update supportFootToRoot with the distance vector from the WRF
    m_robotInterface.computeJacobian(q_rad.data(), supportFootToRoot, neckID, jacobianNeck.data());
    
    // TODO Compute support foot to robot neck forward kinematics
    // neck forward kinematics
    
    // TODO Compute neck velocity
    // neck velocity
    
    bool ret = false;
    return ret;
}

bool WholeBodyNeckVelocityThread::computeSupportFootToRoot(FOOT supportFoot,
							   wbi::Frame& supportFootToRoot) {
  bool ret = false;
  int neckID;
  int footEEFrame;
  yarp::sig::Vector q_rad(m_robotInterface.getDoFs());
  
  retrieveFootEEFrame(supportFoot, footEEFrame);
  m_robotInterface.getFrameList().idToIndex(std::string("neck").c_str(), neckID);
  
  // The following retrieves the rototranslation from root to supportFoot
  wbi::Frame rootToSupportFoot;
  m_robotInterface.computeH(q_rad.data(), wbi::Frame(), footEEFrame, rootToSupportFoot);
  
  // Further rotates it to align z axis with gravity
  rootToSupportFoot = rootToSupportFoot*lastRot;
  
  // Then we invert it to get supportFootToRoot
  supportFootToRoot = rootToSupportFoot.setToInverse();
  return ret;
}

void WholeBodyNeckVelocityThread::retrieveFootEEFrame(FOOT supportFoot,  
						      int& footEEFrame) {
  if (supportFoot == LEFT_FOOT) {
    m_robotInterface.getFrameList().idToIndex("l_sole", footEEFrame);
  } else {
    if (supportFoot == RIGHT_FOOT) { 
      m_robotInterface.getFrameList().idToIndex("r_sole", footEEFrame);
    } else {
      std::cerr << "Support foot is not a valid value" << std::endl;
    }
  }  
}

