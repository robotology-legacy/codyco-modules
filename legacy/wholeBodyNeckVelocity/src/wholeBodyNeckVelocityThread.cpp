#include "wholeBodyNeckVelocityThread.h"

using namespace yarp::math;
using namespace std;

WholeBodyNeckVelocityThread::WholeBodyNeckVelocityThread(wbi::wholeBodyInterface& robotInterface, int rate, string local, FOOT swingingFoot, FOOT WRF, string robot)
: RateThread(rate), 
  m_robotInterface(robotInterface),
  m_neckVelocityPort(0),
  m_local(local),
  m_swingingFoot(swingingFoot),
  m_WRF(WRF),
  m_robot(robot)
{
  m_lastRot.R = wbi::Rotation (0, 0, 1, 0, -1, 0, 1, 0, 0);
  m_swingingFootContactState = true;
  m_doubleSupportState = true;
  m_worldToRootRotoTrans = wbi::Frame(); // Initializes m_worldToRootRotoTrans with the identity matrix
  
  //BEGINS PORTS SECTION >>
  //  Port that will stream the neck velocity
  m_neckVelocityPort = new yarp::os::BufferedPort<yarp::os::Bottle>;

  cout << "DEBUG local is: " << m_local <<  std::endl;   //FIXME
  if(!m_neckVelocityPort || !m_neckVelocityPort->open(("/" + m_local + "/neckVelocity" + ":o").c_str())) {
      std::cout << endl << endl << endl;
      std::cout << "ERR Could not open port to stream neck velocity  ####" << std::endl;
  }  
  
  // Left and right leg ports reading from FT sensors
  m_leftLegFTSensorPort = new yarp::os::BufferedPort<yarp::sig::Vector>;
  m_rightLegFTSensorPort = new yarp::os::BufferedPort<yarp::sig::Vector>;
    
  // Opening left foot sensors readings input port
  if (!m_leftLegFTSensorPort || !m_leftLegFTSensorPort->open(string("/" + m_local + "/leftLegFTSensor:i").c_str())) {
      std::cout << "ERR Could not open port to receive left leg FT readings" << std::endl;
      return;
  }
  // Connecting left leg input port to left foot FT sensors port
  if (!yarp::os::Network::connect(string("/" + m_robot + "/left_foot/analog:o").c_str(), string("/" + m_local + "/leftLegFTSensor:i").c_str())) {
      std::cout << "ERR Could not connect to left foot FT sensor. Is the simulator running? or the platform reachable?" <<  std::endl;
      return;
  }
  // Opening right foot sensors readigs input port
  if (!m_rightLegFTSensorPort || !m_rightLegFTSensorPort->open(string("/" + m_local + "/rightLegFTSensor:i").c_str())) {
      std::cout << "ERR FATAL ERROR Could not open port to receive left leg FT readings" << std::endl;
      return;
  }
  // Connecting right foot input port to right foot FT sensors port
  if (!yarp::os::Network::connect(string("/" + m_robot + "/right_foot/analog:o").c_str(), string("/" + m_local + "/rightLegFTSensor:i").c_str())) {
      std::cout << "ERR FATAL ERROR Could not connect to right foot FT sensor. Is the simulator running? or the platform reachable?" <<  std::endl;
      return;
  }
  
  
  //ENDS PORTS SECTION <<
  
}

WholeBodyNeckVelocityThread::~WholeBodyNeckVelocityThread()
{    
    if (m_neckVelocityPort) {
        m_neckVelocityPort->interrupt();
        m_neckVelocityPort->close();
        m_neckVelocityPort = NULL;
    } else 
        cerr << "ERR Could not close velocity port" << endl;
    if( m_leftLegFTSensorPort) {
        m_leftLegFTSensorPort->interrupt();
        m_leftLegFTSensorPort->close();
        m_leftLegFTSensorPort = NULL;
    } else
        cerr << "ERR Could not close left leg FT sensor" << endl;
    if (m_rightLegFTSensorPort) { 
        m_rightLegFTSensorPort->interrupt();
        m_rightLegFTSensorPort->close();
        m_rightLegFTSensorPort = NULL;
    }  else
        cerr << "ERR Could not close right leg FT sensor" << endl;
}

void WholeBodyNeckVelocityThread::run()
{
    //BEGINS REAL IMPLEMENTATION
    // Is robot in double support phase
    if (isInDoubleSupport()) {
        if (enteredDoubleSupport()) {
           // Change supportFoot
           m_supportFoot = static_cast<FOOT>(!m_supportFoot);
           //TODO test computeDistanceFromPreviousTocurrentSupport
           computeDistanceFromPreviousToCurrentSupport();
           //TODO test updateWorldToSupportFoot
           updateWorldToSupportFoot(m_distanceFromPreviousToCurrentSupport.data());
        }
        updateWorldToRoot();
        computeNeckVelocity();
    } else {
        if (isInSingleSupport()) {
            m_doubleSupportState = false;
            updateWorldToRoot();
            computeNeckVelocity();
        } else {
            cout << "ERR FATAL ERROR The robot must have fallen or feet sensors are not reachable" << endl;
            return;
        }
    }
    //ENDS REAL IMPLEMENTATION
    
    
    
    
    
    // Distance from previous base TODO This must be implemented and updated every cycle
    yarp::sig::Vector distanceToPreviousBase(3);
    distanceToPreviousBase.zero();

    //BEGINS neck velocity computation
    Eigen::VectorXd neckVelocity(6);
    std::cout << "DEBUG About to compute neck velocity with: " << std::endl; //FIXME
    std::cout << "DEBUG m_supportFoot: " << m_supportFoot << std::endl;          //FIXME
//     std::cout << "DEBUG distanceToPreviousBase: " << distanceToPreviousBase.toString() << std::endl; //FIXME
    if(!computeNeckVelocity(neckVelocity, distanceToPreviousBase.data())) {
      std::cerr << "ERR computing neck velocity!" << std::endl;
      return;
    }
    cout << "DEBUG neckVelocity from run(): " << endl << neckVelocity << endl; //FIXME
    //ENDS neck velocity computation
    
    //BEGINS Has swinging foot contact state changed? Initialized as 'true' (in contact)
    if (hasSwingingFootContactChanged()) {
        if (!isFirstContactChange()) {
            // Change support foot
            m_supportFoot = static_cast<FOOT> (!m_supportFoot);
        }
    }
        
    //ENDS swinging foot contact state
    
    //BEGINS If it's not the first time a foot has been lifted, change support and compute worldToSupportFoot
    //ENDS
        
    //BEGINS Stream to port
    m_timeStamp.update();
    yarp::os::Bottle &bot = m_neckVelocityPort->prepare();
    bot.clear();
    for (int i = 0; i < 6; i++)
        bot.addDouble(neckVelocity[i]);
    m_neckVelocityPort->setEnvelope(m_timeStamp);
    m_neckVelocityPort->write();
    //END
}

bool WholeBodyNeckVelocityThread::threadInit()
{
    if (m_WRF == !m_swingingFoot) {
        cout << "DEBUG Initial WRF: " << m_WRF << " is different from initial swinging foot: " << m_swingingFoot << endl;
        m_supportFoot = static_cast<FOOT>(!m_swingingFoot);
    }
    m_worldToSupportFoot.resize(3, 0.0);
    m_distanceFromPreviousToCurrentSupport.resize(3, 0.0);
    return m_robotInterface.init();

}

void WholeBodyNeckVelocityThread::threadRelease()
{
    
}

bool WholeBodyNeckVelocityThread::computeNeckVelocity(Eigen::VectorXd& neckVelocity,
                                                      double* distanceToPreviousBase)
{
    cout << endl; //FIXME
    cout << "******* COMPUTING NECK VELOCITY *******" << endl; //FIXME
    wbi::Frame supportFootToRoot;
    computeSupportFootToRoot(supportFootToRoot);
//     cout << "DEBUG supportFootToRoot: " << supportFootToRoot.toString() << endl; //FIXME

    // Getting joint velocities FIXME Do this in run() and update private variable
    Eigen::VectorXd qd(m_robotInterface.getDoFs());
    m_robotInterface.getEstimates(wbi::ESTIMATE_JOINT_VEL, qd.data());
//     cout << "DEBUG q_rad: " << q_rad << endl; //FIXME

    // Retrieve neck id
    Eigen::MatrixXd jacobianNeck(6, m_robotInterface.getDoFs()+6);
//     Eigen::Matrix<double, Eigen::Dynamic, Eigen::RowMajor> jacobianNeck;
    int neckID = -1;
    if(!m_robotInterface.getFrameList().idToIndex("chest", neckID)) {
        cout << "ERR neck_1 index not found" << endl;
        return false;
    }
    
    // Implement updateWorldToSupportFoot
    updateWorldToSupportFoot(distanceToPreviousBase);
//     cout << "DEBUG Updated worldToSupportFoot: " << m_worldToSupportFoot.toString() << endl; //FIXME
    
    wbi::Frame worldToRootRotoTrans;
    // Assign to worldToRootRotoTrans the same orientation as supportFootToRoot (with the additional rotation to align z with gravity)
    worldToRootRotoTrans.R = supportFootToRoot.R;
    // Here we update  the translational part of this rototranslation
    for (int i=0; i<3; i++)
        worldToRootRotoTrans.p[i] = m_worldToSupportFoot[i] + supportFootToRoot.p[i];
//     cout << "DEBUG updated worldToRootRotoTrans " <<  endl << worldToRootRotoTrans.toString() << endl; //FIXME
    // Compute Jacobian wrt WRF
    m_robotInterface.computeJacobian(qd.data(), worldToRootRotoTrans, neckID, jacobianNeck.data());
//     cout << "DEBUG neck Jacobian: " << endl << jacobianNeck << endl; //FIXME
    // Compute neck velocity 
    // neck velocity
    Eigen::VectorXd tmpNeckVelocity(6);
    tmpNeckVelocity = jacobianNeck*qd;
    cout << "DEBUG Neck velocity: " << endl << tmpNeckVelocity << endl; //FIXME
    neckVelocity = tmpNeckVelocity;
    cout << "DEBUG Reached end of comuteNeckVelocity, returning: " << endl << neckVelocity;
    
    return true;
}

bool WholeBodyNeckVelocityThread::computeRootToFoot ( wbi::Frame& rootToFoot, FOOT foot )
{
    bool ret = false;
    int footEEFrame;
    
    yarp::sig::Vector q_rad(m_robotInterface.getDoFs());
    q_rad.zero();
    
    if(!m_robotInterface.getEstimates(wbi::ESTIMATE_JOINT_POS, q_rad.data())) {
        cout << "ERR Retrieving joint angles" << endl;
        return false;
    }
    
    // Retrieve desired foot EE frame
    if (foot == LEFT_FOOT) {
        m_robotInterface.getFrameList().idToIndex("l_sole", footEEFrame);
    } else {
        if (foot == RIGHT_FOOT) { 
            m_robotInterface.getFrameList().idToIndex("r_sole", footEEFrame);
        } else {
            std::cerr << "ERR Support foot is not a valid value" << std::endl;
            return false;
        }
    }
    
    // The following retrieves the rototranslation from root to foot
    wbi::Frame tmpRootToFoot;
    m_robotInterface.computeH(q_rad.data(), wbi::Frame(), footEEFrame, tmpRootToFoot);
    
    // Further rotates it to align z axis with gravity
    tmpRootToFoot = tmpRootToFoot*m_lastRot;
    
    rootToFoot = &tmpRootToFoot;
    cout << "DEBUG rototranslation from root to foot: " << endl << rootToFoot.toString() << endl; //FIXME
}

bool WholeBodyNeckVelocityThread::computeSupportFootToRoot( wbi::Frame& supportFootToRoot ) {
    bool ret = true;
//     int neckID;
    int footEEFrame;
    yarp::sig::Vector q_rad(m_robotInterface.getDoFs());
    q_rad.zero();
  
    if(!m_robotInterface.getEstimates(wbi::ESTIMATE_JOINT_POS, q_rad.data())) {
        cout << "ERR Retrieving joint angles" << endl;
        return false;
    }
    cout << "DEBUG q_rad in computeSupportFootToRoot: " << q_rad.toString() << endl; //FIXME
  
    // Retrieve current support foot EE frame
    retrieveFootEEFrame(footEEFrame);
//     if(!m_robotInterface.getFrameList().idToIndex(std::string("chest").c_str(), neckID)) {
//         cout << "ERR neck_1 not found" << endl;
//         return false;
//     }
  
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

void WholeBodyNeckVelocityThread::retrieveFootEEFrame(int& footEEFrame) {
  if (m_supportFoot == LEFT_FOOT) {
    m_robotInterface.getFrameList().idToIndex("l_sole", footEEFrame);
  } else {
    if (m_supportFoot == RIGHT_FOOT) { 
      m_robotInterface.getFrameList().idToIndex("r_sole", footEEFrame);
    } else {
      std::cerr << "Support foot is not a valid value" << std::endl;
    }
  }
}

void WholeBodyNeckVelocityThread::updateWorldToSupportFoot(double* distanceFromPreviousSupport)
{
    yarp::sig::Vector tmpDistanceFromPreviousBase(3);
    tmpDistanceFromPreviousBase.zero();
    for (int i=0; i<tmpDistanceFromPreviousBase.length(); i++)
        tmpDistanceFromPreviousBase[i] = distanceFromPreviousSupport[i];
//     tmpDistanceToPreviousBase.setSubvector(0,tmpDistanceToPreviousBase);
//     cout << "DEBUG tmpDistanceToPreviousBase: " << tmpDistanceToPreviousBase.toString() << endl; //FIXME
    //TODO m_worldToSupportFoot is not being initialized
//     cout << "DEBUG m_worldToSupportFoot: " << m_worldToSupportFoot.toString() << endl; //FIXME
    //  No rotation needed because WRF and support feet have the same orientation
    m_worldToSupportFoot = m_worldToSupportFoot + tmpDistanceFromPreviousBase;
}

bool WholeBodyNeckVelocityThread::isInDoubleSupport()
{
    bool ret = false;
    yarp::sig::Vector* leftLegFTMeasurement  = m_leftLegFTSensorPort->read();
    yarp::sig::Vector* rightLegFTMeasurement = m_rightLegFTSensorPort->read();
    
    if ((leftLegFTMeasurement->operator[](2) < THRESHOLD_LEFT_FOOT_OFF) && (rightLegFTMeasurement->operator[](2) < THRESHOLD_RIGHT_FOOT_OFF))
        ret = true;
    
    return ret;
}

bool WholeBodyNeckVelocityThread::isInSingleSupport()
{
    bool ret = false;
    yarp::sig::Vector* leftLegFTMeasurement  = m_leftLegFTSensorPort->read();
    yarp::sig::Vector* rightLegFTMeasurement = m_rightLegFTSensorPort->read();
    
    if (    (leftLegFTMeasurement->operator[](2) < THRESHOLD_LEFT_FOOT_OFF) && !(rightLegFTMeasurement->operator[](2) < THRESHOLD_RIGHT_FOOT_OFF)
        || !(leftLegFTMeasurement->operator[](2) < THRESHOLD_LEFT_FOOT_OFF) &&  (rightLegFTMeasurement->operator[](2) < THRESHOLD_RIGHT_FOOT_OFF)  )
        ret = true;
    
    return ret;
}

bool WholeBodyNeckVelocityThread::enteredDoubleSupport()
{
    bool ret = false;
    if ( (m_doubleSupportState == false) && (isInDoubleSupport()) ) {
        m_doubleSupportState = true;
        ret = true;
    }
    
    return ret;

}

bool WholeBodyNeckVelocityThread::updateWorldToRoot()
{
    bool ret = false;    
    return ret;
    

}

bool WholeBodyNeckVelocityThread::hasSwingingFootContactChanged()
{

}

bool WholeBodyNeckVelocityThread::isFirstContactChange()
{

}

void WholeBodyNeckVelocityThread::computeDistanceFromPreviousSupportToCurrentSupport()
{
    //TODO TODO TODO CHECK THE WHOLE ALGORITHM TODO TODO TODO !!!!!!!!!!!!!!!
    // Add up vectors 'current support foot to root' + 'root to the other foot'
    // The following lines compute the rototranslation from the current support foot to root.
    wbi::Frame& supportFootToRootRotoTrans;
    wbi::Frame& rootToFootRotoTrans;
    FOOT foot = static_cast<FOOT> (!m_supportFoot);
    computeSupportFootToRoot(supportFootToRootRotoTrans);
    wbi::Frame rootToSupportFootRotoTrans = supportFootToRootRotoTrans.getInverse();
    computeRootToFoot(rootToFootRotoTrans, foot);
    wbi::Frame footToRootRotoTrans = rootToFootRotoTrans.getInverse();
    
    // What I actually want is the distance vector from the previous support to the current one.
    yarp::sig::Vector rootToSupportFootVec(3, rootToSupportFootRotoTrans.p);
    yarp::sig::Vector footToRootVec(3, footToRootRotoTrans.p);
    
    m_distanceFromPreviousToCurrentSupport = footToRootRotoTrans.R*rootToSupportFootVec + footToRootVec;    
}







