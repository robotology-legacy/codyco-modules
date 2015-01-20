#include "wholeBodyNeckVelocityThread.h"

using namespace yarp::math;

WholeBodyNeckVelocityThread::WholeBodyNeckVelocityThread(wbi::wholeBodyInterface& robotInterface, int rate): RateThread(rate), m_robotInterface(robotInterface)
{
  m_lastRot.R = wbi::Rotation (0, 0, 1, 0, -1, 0, 1, 0, 0);
  m_worldToSupportFoot;
}

WholeBodyNeckVelocityThread::~WholeBodyNeckVelocityThread()
{
    
}

void WholeBodyNeckVelocityThread::run()
{

  // Support foot TODO This must be updated every cycle according to F/T sensors input
  FOOT supportFoot = LEFT_FOOT;
  
  // Distance from previous base TODO This must be updated every cycle
  yarp::sig::Vector distanceToPreviousBase(3);
  distanceToPreviousBase.zero();
 
  // compute neck velocity
  yarp::sig::Vector neckVelocity(3);
  neckVelocity.zero();
  if(!computeNeckVelocity(neckVelocity.data(), supportFoot, distanceToPreviousBase.data())) {
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

bool WholeBodyNeckVelocityThread::computeNeckVelocity(double* neckVelocity,
                                                      FOOT    supportFoot,
                                                      double* distanceToPreviousBase)
{
    wbi::Frame supportFootToRoot;
    computeSupportFootToRoot(supportFoot, supportFootToRoot);

    // Getting joint angles FIXME Do this in run() and update private variable
    Eigen::VectorXd q_rad(m_robotInterface.getDoFs());
    m_robotInterface.getEstimates(wbi::ESTIMATE_JOINT_POS, q_rad.data());

    // Retrieve neck id
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::RowMajor> jacobianNeck;
    int neckID = -1;
    m_robotInterface.getFrameList().idToIndex("neck", neckID);
    
    // TODO Implement updateWorldToSupportFoot
    updateWorldToSupportFoot(distanceToPreviousBase);
    
    // TODO Pass worldToRootRotoTrans to computeJacobian
    wbi::Frame worldToRootRotoTrans;
    // Assign to worldToRootRotoTrans the same orientation as supportFootToRoot (with the additional rotation to align z with gravity)
    worldToRootRotoTrans.R = supportFootToRoot.R;
    // TODO Here we update  the translational part of this rototranslation
    for (int i=0; i<3; i++)
        worldToRootRotoTrans.p[i] = m_worldToSupportFoot[i] + supportFootToRoot.p[i];
    // Compute Jacobian wrt WRF
    m_robotInterface.computeJacobian(q_rad.data(), worldToRootRotoTrans, neckID, jacobianNeck.data());

    // Compute neck velocity
    // neck velocity
    Eigen::VectorXd tmpNeckVelocity(6);
    tmpNeckVelocity = jacobianNeck*q_rad;
    neckVelocity = tmpNeckVelocity.data();
    
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
  rootToSupportFoot = rootToSupportFoot*m_lastRot;
  
  // Then we invert it to get supportFootToRoot
  supportFootToRoot = rootToSupportFoot.setToInverse();
  ret = true;
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

void WholeBodyNeckVelocityThread::updateWorldToSupportFoot(double* distanceToPreviousBase)
{
    yarp::sig::Vector tmpDistanceToPreviousBase(3);
    tmpDistanceToPreviousBase.setSubvector(0,tmpDistanceToPreviousBase);
    m_worldToSupportFoot = m_worldToSupportFoot + tmpDistanceToPreviousBase;
}
