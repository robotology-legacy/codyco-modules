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


class rangeCheckSafe
{
public:
    Vector qMin;
    Vector qMax;
    Vector qMed;
    Vector qSpan;
    
    Vector dqMin;
    Vector dqMax;
    
    Vector ddqMin;
    Vector ddqMax;
    
    Vector tauMin;
    Vector tauMax;
    
    double tol;
    double tolDq;
    double tolDdq;
    double tolTau;
    
public:
    
    
    rangeCheckSafe(Vector _qMin, Vector _qMax, double _tol = 1.0);
    rangeCheckSafe(const Vector& _qMin, const Vector& _qMax,
                   const Vector& _dqMin, const Vector& _dqMax,
                   const Vector& _ddqMin, const Vector& _ddqMax,
                   const Vector& _tauMin, const Vector& _tauMax,
                   double _tol = 1.0, double _tolDq = 1.0, double _tolDdq = 1.0, double _tolTau = 1.0);
    ~rangeCheckSafe();
    
    //
    
    void setRangeCheckSafe(const Vector& _qMin, const Vector& _qMax,
                           const Vector& _dqMin, const Vector& _dqMax,
                           const Vector& _ddqMin, const Vector& _ddqMax,
                           const Vector& _tauMin, const Vector& _tauMax,
                           double _tol = 1.0, double _tolDq = 1.0, double _tolDdq = 1.0, double _tolTau = 1.0);
    void isAtBoundaries(const Vector &q, Vector &m);
    void isAtBoundariesVel(const Vector &dq, Vector &m);
    void isAtBoundariesTorque(const Vector &tau, Vector &m);
    
};


#endif
