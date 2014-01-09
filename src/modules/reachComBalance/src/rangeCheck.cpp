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


//=========================
//
//   RANGE CHECK SAFE
//
//=========================


//-------------------------------------------------------------------
rangeCheckSafe::rangeCheckSafe(const Vector& _qMin, const Vector& _qMax, const Vector& _dqMin, const Vector& _dqMax, const Vector& _ddqMin, const Vector& _ddqMax, const Vector& _tauMin, const Vector& _tauMax, double _tol, double _tolDq, double _tolDdq, double _tolTau)
:qMin(_qMin),qMax(_qMax),dqMin(_dqMin), dqMax(_dqMax), ddqMin(_ddqMin), ddqMax(_ddqMax), tauMin(_tauMin), tauMax(_tauMax), tol(_tol),tolDq(_tolDq),tolDdq(_tolDdq),tolTau(_tolTau)
{
    qMed.resize(qMin.size());
    for(int i = 0; i < qMin.size(); i++)
        qMed(i) = 0.5 * (qMin(i) + qMax(i));
    
    qSpan.resize(qMin.size());
    for(int i = 0; i < qMin.size(); i++)
        qSpan(i) = qMax(i) - qMin(i);
    
}
//-------------------------------------------------------------------
rangeCheckSafe::rangeCheckSafe(Vector _qMin, Vector _qMax, double _tol):qMin(_qMin),qMax(_qMax),tol(_tol)
{
    double _tolDq=0.5;
    double _tolDdq=0.01;
    double _tolTau=0.01;
    
    int dim=_qMin.size();
    
    Vector _dqMin(dim,0.0);
    Vector _dqMax(dim,10.0);
    
    Vector _ddqMin(dim,1.0);
    Vector _ddqMax(dim,1.0);
    
    Vector _tauMin(dim,-10.0);
    Vector _tauMax(dim,10.0);
    
    setRangeCheckSafe(_qMin, _qMax,
                      _dqMin, _dqMax,
                      _ddqMin, _ddqMax,
                      _tauMin, _tauMax,
                      _tol, _tolDq, _tolDdq, _tolTau);
    
}

//-------------------------------------------------------------------
void rangeCheckSafe::setRangeCheckSafe(const Vector& _qMin, const Vector& _qMax,
                                       const Vector& _dqMin, const Vector& _dqMax,
                                       const Vector& _ddqMin, const Vector& _ddqMax,
                                       const Vector& _tauMin, const Vector& _tauMax,
                                       double _tol, double _tolDq, double _tolDdq, double _tolTau)
{
    qMin = _qMin;
    qMax = _qMax;
    dqMin = _dqMin;
    dqMax = _dqMax;
    ddqMin = _ddqMin;
    ddqMax = _ddqMax;
    tauMin = _tauMin;
    tauMax = _tauMax;
    tol = _tol;
    tolDq = _tolDq;
    tolDdq = _tolDdq;
    tolTau = _tolTau;
    
    
    qMed.resize(qMin.size());
    for(int i = 0; i < qMin.size(); i++)
        qMed(i) = 0.5 * (qMin(i) + qMax(i));
    
    qSpan.resize(qMin.size());
    for(int i = 0; i < qMin.size(); i++)
        qSpan(i) = qMax(i) - qMin(i);
    
}
//-------------------------------------------------------------------
rangeCheckSafe::~rangeCheckSafe()
{
}
//-------------------------------------------------------------------
void rangeCheckSafe::isAtBoundaries(const Vector& q, Vector &mask)
{
    mask.zero();
    
    for(int i = 0; i < qMin.size(); i++)
        if(q(i) < qMed(i) - 0.5 * qSpan(i) * tol)
            mask(i) = -1.0;
    
    for(int i = 0; i < qMax.size(); i++)
        if(q(i) > qMed(i) + 0.5 * qSpan(i) * tol)
            mask(i) =  1.0;
}
//-------------------------------------------------------------------
