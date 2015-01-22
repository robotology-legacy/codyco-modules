#ifndef WHOLEBODYNECKVELOCITYTHREAD_H
#define WHOLEBODYNECKVELOCITYTHREAD_H

#include <yarp/os/RateThread.h>
#include <yarp/math/Math.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <wbi/wholeBodyInterface.h>
#include <Eigen/Core>
#include <constants.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>

class WholeBodyNeckVelocityThread : public yarp::os::RateThread {
private:
  wbi::wholeBodyInterface& m_robotInterface;
  int                      m_rate;
  wbi::Frame               m_lastRot;
  yarp::sig::Vector        m_worldToSupportFoot;
  yarp::os::BufferedPort<yarp::os::Bottle> *m_neckVelocityPort;
  std::string              m_local;
  yarp::os::Stamp          m_timeStamp;
public:
  WholeBodyNeckVelocityThread(wbi::wholeBodyInterface& robotInterface, int rate, std::string local);
  virtual ~WholeBodyNeckVelocityThread();
  //BEGINS RateThread inherited methods
  virtual bool threadInit();
  virtual void threadRelease();
  virtual void run();
  //ENDS RateThread inherited methods
  bool computeNeckVelocity(Eigen::VectorXd& neckVelocity, FOOT supportFoot, double* distanceToPreviousBase);
  bool computeSupportFootToRoot(FOOT supportFoot, wbi::Frame& supportFootToRoot);
  void retrieveFootEEFrame(FOOT supportFoot, int& footEEFrame);
  void updateWorldToSupportFoot(double* distanceToPreviousBase);
};
  

#endif