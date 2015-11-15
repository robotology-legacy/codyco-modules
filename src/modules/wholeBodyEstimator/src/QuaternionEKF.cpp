#include "QuaternionEKF.h"

REGISTERIMPL(QuaternionEKF);

using namespace yarp::os;

bool QuaternionEKF::init(ResourceFinder &rf, wbi::iWholeBodySensors *wbs)
{
    return true;
}

void QuaternionEKF::run()
{
    
}

void QuaternionEKF::release()
{
    
}