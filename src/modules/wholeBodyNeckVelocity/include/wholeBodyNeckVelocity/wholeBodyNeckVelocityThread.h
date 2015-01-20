#ifndef WHOLEBODYNECKVELOCITYTHREAD_H
#define WHOLEBODYNECKVELOCITYTHREAD_H

#include <yarp/os/RateThread.h>
#include <yarp/math/Math.h>
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
public:
  WholeBodyNeckVelocityThread(wbi::wholeBodyInterface& robotInterface, int rate);
  virtual ~WholeBodyNeckVelocityThread();
  //BEGINS RateThread inherited methods
  virtual bool threadInit();
  virtual void threadRelease();
  virtual void run();
  //ENDS RateThread inherited methods
  bool computeNeckVelocity(double* neckVelocity, FOOT supportFoot, double* distanceToPreviousBase);
  bool computeSupportFootToRoot(FOOT supportFoot, wbi::Frame& supportFootToRoot);
  void retrieveFootEEFrame(FOOT supportFoot, int& footEEFrame);
  void updateWorldToSupportFoot(double* distanceToPreviousBase);
};
  

#endif