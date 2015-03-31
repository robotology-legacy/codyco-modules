#include "DynamicConstraint.h"

#include <yarp/sig/Vector.h>
#include <iCub/ctrl/minJerkCtrl.h>

using namespace codyco::torquebalancing;

struct DynamicConstraintPrivate {
    unsigned active;
    iCub::ctrl::minJerkTrajGen *transitionSmoother;
    yarp::sig::Vector innerValue;
    yarp::sig::Vector lastComputedValue;

    DynamicConstraintPrivate()
    : active(0)
    , transitionSmoother(0)
    , innerValue(1, 0.0)
    , lastComputedValue(1, 0.0) {}

    ~DynamicConstraintPrivate() {
        if (transitionSmoother) {
            delete transitionSmoother;
            transitionSmoother = 0;
        }
    }
};

DynamicConstraint::DynamicConstraint()
: m_implementation(new DynamicConstraintPrivate()) {}

DynamicConstraint::~DynamicConstraint()
{
    DynamicConstraintPrivate *privateData = static_cast<DynamicConstraintPrivate*>(m_implementation);

    if (privateData) {
        delete privateData;
        m_implementation = 0;
    }
}

bool DynamicConstraint::init(bool isConstraintActiveAtInit, double timeStep, double transitionTime)
{
    DynamicConstraintPrivate *privateData = static_cast<DynamicConstraintPrivate*>(m_implementation);
    if (!privateData) return false;
    //already initialized
    if (privateData->transitionSmoother) return false;

    privateData->transitionSmoother = new iCub::ctrl::minJerkTrajGen(1, timeStep, transitionTime);
    privateData->innerValue(0) = isConstraintActiveAtInit ? 1.0 : 0.0;
    privateData->lastComputedValue(0) = privateData->innerValue(0);
    if (privateData->transitionSmoother) privateData->transitionSmoother->init(privateData->innerValue);
    return privateData->transitionSmoother != NULL;
}

bool DynamicConstraint::isActive() const
{
    DynamicConstraintPrivate *privateData = static_cast<DynamicConstraintPrivate*>(m_implementation);
    if (!privateData) return false;
    return privateData->active != 0;
}

bool DynamicConstraint::isActiveWithThreshold(double threshold) const
{
    DynamicConstraintPrivate *privateData = static_cast<DynamicConstraintPrivate*>(m_implementation);
    if (!privateData) return false;
    return privateData->lastComputedValue(0) >= threshold;
}

double DynamicConstraint::continuousValue() const
{
    DynamicConstraintPrivate *privateData = static_cast<DynamicConstraintPrivate*>(m_implementation);
    if (!privateData || !privateData->transitionSmoother) return false;
    return privateData->lastComputedValue(0);
}

void DynamicConstraint::updateStateInterpolation()
{
    DynamicConstraintPrivate *privateData = static_cast<DynamicConstraintPrivate*>(m_implementation);
    if (!privateData || !privateData->transitionSmoother) return;
    privateData->transitionSmoother->computeNextValues(privateData->innerValue);
    privateData->lastComputedValue = privateData->transitionSmoother->getPos();
}

void DynamicConstraint::activate()
{
    DynamicConstraintPrivate *privateData = static_cast<DynamicConstraintPrivate*>(m_implementation);
    if (!privateData || !privateData->transitionSmoother) return;
    privateData->active = 1;
    privateData->innerValue = 1.0;
}

void DynamicConstraint::deactivate()
{
    DynamicConstraintPrivate *privateData = static_cast<DynamicConstraintPrivate*>(m_implementation);
    if (!privateData || !privateData->transitionSmoother) return;
    privateData->active = 0;
    privateData->innerValue = 0.0;

}
