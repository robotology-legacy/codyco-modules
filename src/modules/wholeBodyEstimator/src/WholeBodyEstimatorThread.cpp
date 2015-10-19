#include "WholeBodyEstimatorThread.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace wbi;

WholeBodyEstimatorThread::WholeBodyEstimatorThread (ResourceFinder &rf, iWholeBodySensors* wbs, int period) : RateThread(period),
                                                                                                              m_rfCopy(rf),
                                                                                                              m_wbs(wbs)
{
	
}

bool WholeBodyEstimatorThread::threadInit()
{
    
    m_floatingBaseLeggedOdometry = new LeggedOdometry;
    m_floatingBaseLeggedOdometry->init(m_rfCopy, m_wbs);
    
    return true;
}

void WholeBodyEstimatorThread::run()
{
    m_floatingBaseLeggedOdometry->run();
}

void WholeBodyEstimatorThread::threadRelease()
{
    m_floatingBaseLeggedOdometry->release();
}