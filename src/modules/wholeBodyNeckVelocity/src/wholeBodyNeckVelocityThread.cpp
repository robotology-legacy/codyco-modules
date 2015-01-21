#include "wholeBodyNeckVelocityThread.h"
#include <Eigen/src/Eigenvalues/ComplexSchur.h>

using namespace yarp::math;
using namespace std;

WholeBodyNeckVelocityThread::WholeBodyNeckVelocityThread(wbi::wholeBodyInterface& robotInterface, int rate): RateThread(rate), m_robotInterface(robotInterface)
{
  m_lastRot.R = wbi::Rotation (0, 0, 1, 0, -1, 0, 1, 0, 0);
}

WholeBodyNeckVelocityThread::~WholeBodyNeckVelocityThread()
{
    
}

void WholeBodyNeckVelocityThread::run()
{

//     std::cout << "DEBUG In thread run!..\n"; //FIXME
    // Support foot TODO This must be updated every cycle according to F/T sensors input
    FOOT supportFoot = LEFT_FOOT;
  
    // Distance from previous base TODO This must be implemented and updated every cycle
    yarp::sig::Vector distanceToPreviousBase(3);
    distanceToPreviousBase.zero();
 
    // compute neck velocity
    Eigen::VectorXd neckVelocity(6);
    std::cout << "DEBUG About to compute neck velocity with: " << std::endl; //FIXME
    std::cout << "DEBUG supportFoot: " << supportFoot << std::endl;          //FIXME
//     std::cout << "DEBUG distanceToPreviousBase: " << distanceToPreviousBase.toString() << std::endl; //FIXME
    if(!computeNeckVelocity(neckVelocity, supportFoot, distanceToPreviousBase.data())) {
      std::cerr << "ERR computing neck velocity!" << std::endl;
      return;
    }
    cout << "DEBUG neckVelocity from run(): " << endl << neckVelocity << endl; //FIXME
}

bool WholeBodyNeckVelocityThread::threadInit()
{
    m_worldToSupportFoot.resize(3,0.0);
    return m_robotInterface.init();

}

void WholeBodyNeckVelocityThread::threadRelease()
{
}

bool WholeBodyNeckVelocityThread::computeNeckVelocity(Eigen::VectorXd& neckVelocity,
                                                      FOOT    supportFoot,
                                                      double* distanceToPreviousBase)
{
    cout << endl;
    cout << "******* COMPUTING NECK VELOCITY *******" << endl;
    wbi::Frame supportFootToRoot;
    computeSupportFootToRoot(supportFoot, supportFootToRoot);
//     cout << "DEBUG supportFootToRoot: " << supportFootToRoot.toString() << endl; //FIXME

    // Getting joint angles FIXME Do this in run() and update private variable
    Eigen::VectorXd q_rad(m_robotInterface.getDoFs());
    m_robotInterface.getEstimates(wbi::ESTIMATE_JOINT_POS, q_rad.data());
//     cout << "DEBUG q_rad: " << q_rad << endl; //FIXME

    // Retrieve neck id
    Eigen::MatrixXd jacobianNeck(6, m_robotInterface.getDoFs()+6);
//     Eigen::Matrix<double, Eigen::Dynamic, Eigen::RowMajor> jacobianNeck;
    int neckID = -1;
    m_robotInterface.getFrameList().idToIndex("neck", neckID);
    
    // Implement updateWorldToSupportFoot
    updateWorldToSupportFoot(distanceToPreviousBase);
//     cout << "DEBUG Updated worldToSupportFoot: " << m_worldToSupportFoot.toString() << endl; //FIXME
    
    // TODO Pass worldToRootRotoTrans to computeJacobian
    wbi::Frame worldToRootRotoTrans;
    // Assign to worldToRootRotoTrans the same orientation as supportFootToRoot (with the additional rotation to align z with gravity)
    worldToRootRotoTrans.R = supportFootToRoot.R;
    // TODO Here we update  the translational part of this rototranslation
    for (int i=0; i<3; i++)
        worldToRootRotoTrans.p[i] = m_worldToSupportFoot[i] + supportFootToRoot.p[i];
//     cout << "DEBUG updated worldToRootRotoTrans " <<  endl << worldToRootRotoTrans.toString() << endl; //FIXME
    // Compute Jacobian wrt WRF
    m_robotInterface.computeJacobian(q_rad.data(), worldToRootRotoTrans, neckID, jacobianNeck.data());
//     cout << "DEBUG neck Jacobian: " << endl << jacobianNeck << endl; //FIXME
    // Compute neck velocity
    // neck velocity
    Eigen::VectorXd tmpNeckVelocity(6);
    tmpNeckVelocity = jacobianNeck*q_rad;
    cout << "DEBUG Neck velocity: " << endl << tmpNeckVelocity << endl; //FIXME
    neckVelocity = tmpNeckVelocity;
    cout << "DEBUG Reached end of comuteNeckVelocity, returning: " << endl << neckVelocity;
    
    return true;
}

bool WholeBodyNeckVelocityThread::computeSupportFootToRoot(FOOT supportFoot,
                                                           wbi::Frame& supportFootToRoot) {
    bool ret = true;
    int neckID;
    int footEEFrame;
    yarp::sig::Vector q_rad(m_robotInterface.getDoFs());
    q_rad.zero();
  
    if(!m_robotInterface.getEstimates(wbi::ESTIMATE_JOINT_POS, q_rad.data())) {
        cout << "ERR Retrieving joint angles" << endl;
        return false;
    }
    cout << "DEBUG q_rad in computeSupportFootToRoot: " << q_rad.toString() << endl; //FIXME
  
    retrieveFootEEFrame(supportFoot, footEEFrame);
    m_robotInterface.getFrameList().idToIndex(std::string("neck").c_str(), neckID);
  
    // The following retrieves the rototranslation from root to supportFoot
    wbi::Frame rootToSupportFoot;
    m_robotInterface.computeH(q_rad.data(), wbi::Frame(), footEEFrame, rootToSupportFoot);
  
    // Further rotates it to align z axis with gravity
    rootToSupportFoot = rootToSupportFoot*m_lastRot;
    cout << "DEBUG rototranslation from root to supportFoot: " << endl << rootToSupportFoot.toString() << endl; //FIXME
  
    // Then we invert it to get supportFootToRoot
    supportFootToRoot = rootToSupportFoot.setToInverse();
    cout << "DEBUG rototranslation from supportFoot to root: " << endl << supportFootToRoot.toString() << endl; //FIXME
    return true;
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
    tmpDistanceToPreviousBase.zero();
    for (int i=0; i<tmpDistanceToPreviousBase.length(); i++)
        tmpDistanceToPreviousBase[i] = distanceToPreviousBase[i];
//     tmpDistanceToPreviousBase.setSubvector(0,tmpDistanceToPreviousBase);
//     cout << "DEBUG tmpDistanceToPreviousBase: " << tmpDistanceToPreviousBase.toString() << endl; //FIXME
    //TODO m_worldToSupportFoot is not being initialized
//     cout << "DEBUG m_worldToSupportFoot: " << m_worldToSupportFoot.toString() << endl; //FIXME
    m_worldToSupportFoot = m_worldToSupportFoot + tmpDistanceToPreviousBase;
}
