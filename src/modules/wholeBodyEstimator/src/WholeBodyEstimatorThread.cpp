#include "WholeBodyEstimatorThread.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

WholeBodyEstimatorThread::WholeBodyEstimatorThread (ResourceFinder &rf, int period) : RateThread(period)
{
	
}

bool WholeBodyEstimatorThread::threadInit()
{
    bool ret = false;
    return ret;
	
}

void WholeBodyEstimatorThread::run()
{
	
}

void WholeBodyEstimatorThread::threadRelease()
{
	bool ret = false;
	return ret;
}