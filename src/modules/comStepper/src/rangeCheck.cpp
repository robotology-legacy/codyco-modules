#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/Stamp.h>
#include <yarp/math/Math.h>
#include <math.h>
#include <stdio.h>

#include "rangeCheck.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::math;
using namespace yarp::dev;
using namespace std;

rangeCheck::rangeCheck(Vector _qMin, Vector _qMax, double tolerance):qMin(_qMin),qMax(_qMax),tol(1.0)
{
    tol = tolerance;
        
    qMed.resize(qMin.size());
    for(int i = 0; i < qMin.size(); i++)
        qMed(i) = 0.5 * (qMin(i) + qMax(i));
    
    qSpn.resize(qMin.size());
    for(int i = 0; i < qMin.size(); i++)
        qSpn(i) = qMax(i) - qMin(i);

}

rangeCheck::~rangeCheck()
{
    
}

void rangeCheck::isAtBoundaries(Vector q, Vector &mask)
{
    mask.zero();
    
    for(int i = 0; i < qMin.size(); i++)
        if(q(i) < qMed(i) - 0.5 * qSpn(i) * tol)
            mask(i) = -1.0;

    for(int i = 0; i < qMax.size(); i++)
        if(q(i) > qMed(i) + 0.5 * qSpn(i) * tol)
            mask(i) =  1.0;
}
