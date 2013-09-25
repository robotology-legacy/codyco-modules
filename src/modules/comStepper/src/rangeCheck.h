#ifndef RANGE_CHECK
#define RANGE_CHECK

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/Stamp.h>
#include <iCub/ctrl/math.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace std;

class rangeCheck
{
public:
    Vector qMin;
    Vector qMax;
    Vector qMed;
    Vector qSpn;
    double tol;
private:
    
public:
    rangeCheck(Vector _qMin, Vector _qMax, double tol = 1.0);
    ~rangeCheck();
    void isAtBoundaries(Vector q, Vector &m);
    
};

#endif
