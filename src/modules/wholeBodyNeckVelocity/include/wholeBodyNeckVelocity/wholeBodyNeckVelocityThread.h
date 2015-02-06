#ifndef WHOLEBODYNECKVELOCITYTHREAD_H
#define WHOLEBODYNECKVELOCITYTHREAD_H

#include <yarp/os/RateThread.h>
#include <yarp/math/Math.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Network.h>
#include <wbi/wholeBodyInterface.h>
#include <Eigen/Core>
#include <constants.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>

class WholeBodyNeckVelocityThread : public yarp::os::RateThread {
private:
  wbi::wholeBodyInterface&                   m_robotInterface;
  int                                        m_rate;
  wbi::Frame                                 m_lastRot;
  yarp::sig::Vector                          m_worldToSupportFoot;
  yarp::os::BufferedPort<yarp::os::Bottle>  *m_neckVelocityPort;
  yarp::os::BufferedPort<yarp::sig::Vector> *m_leftLegFTSensorPort;
  yarp::os::BufferedPort<yarp::sig::Vector> *m_rightLegFTSensorPort;
  std::string                                m_local;
  std::string                                m_robot; 
  yarp::os::Stamp                            m_timeStamp;
  FOOT                                       m_swingingFoot;
  /** WRF stands for world reference frame and corresponds to left or right foot **/
  FOOT                                       m_WRF; 
  FOOT                                       m_supportFoot;
  /** Boolean variable to express the swinging foot contact status. True when swinging foot in contact, False otherwise **/
  bool                                       m_swingingFootContactState;
  bool                                       m_doubleSupportState;
  yarp::sig::Vector                          m_distanceFromPreviousToCurrentSupport;
  /* WRF to ROOT rototranslation. WRF and supportFoot have the same orientation.*/
  wbi::Frame                                 m_worldToRootRotoTrans;
public:
  WholeBodyNeckVelocityThread(wbi::wholeBodyInterface& robotInterface, int rate, std::string local, FOOT swingingFoot, FOOT WRF, std::string robot);
  virtual ~WholeBodyNeckVelocityThread();
  //BEGINS RateThread inherited methods
  virtual bool threadInit();
  virtual void threadRelease();
  virtual void run();
  //ENDS RateThread inherited methods
  bool computeNeckVelocity(Eigen::VectorXd& neckVelocity, double* distanceToPreviousBase);
  bool computeSupportFootToRoot(wbi::Frame& supportFootToRoot);
  bool computeRootToFoot(wbi::Frame& rootToFoot, FOOT foot);
  void retrieveFootEEFrame(int& footEEFrame);
  /** Distance vector from WRF to supportFoot w.r.t. WRF **/
  void updateWorldToSupportFoot(double* distanceFromPreviousToCurrentSupport);
  bool hasSwingingFootContactChanged();
  bool isFirstContactChange();
  /* isInDoubleSupport checks if both sensors are whithin F/T threshold */
  bool isInDoubleSupport();
  /* enteredDoubleSupport checks whether the robot entered the double support phase */
  bool enteredDoubleSupport();
  /* isSingleSupport checks whether the robot is in the single support phase*/
  bool isInSingleSupport();
  /* updateWorldToRoot updates the world to root rototranslation using updateWorldToSupportFoot*/
  bool updateWorldToRoot();
  /* Compute the distance vector from the previous to the current support foot expressed w.r.t. previous support foot */
  void computeDistanceFromPreviousToCurrentSupport();
};
  

#endif