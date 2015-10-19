#include "WholeBodyEstimatorThread.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

WholeBodyEstimatorThread::WholeBodyEstimatorThread (ResourceFinder &rf, int period) : RateThread(period),
                                                                                      m_rfCopy(rf)
{
	
}

bool WholeBodyEstimatorThread::threadInit()
{
    bool ret = false;
    
    m_floatingBaseLeggedOdometry = new LeggedOdometry;
    m_floatingBaseLeggedOdometry->init(m_rfCopy, m_wbs);
    return ret;
	
}

void WholeBodyEstimatorThread::run()
{
    m_floatingBaseLeggedOdometry->run();
}

void WholeBodyEstimatorThread::threadRelease()
{
    m_floatingBaseLeggedOdometry->release();
}