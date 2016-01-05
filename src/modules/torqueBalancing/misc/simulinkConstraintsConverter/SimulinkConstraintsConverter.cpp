#include "SimulinkConstraintsConverter.h"

#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/SharedLibraryClass.h>
#include <cmath>

using namespace yarp::os;
using namespace codyco::torquebalancing;

static bool isFloatEqual(double a, double b, double tol);

YARP_DEFINE_SHARED_SUBCLASS(MonitorObject_there, SimulinkConstraintsConverter, MonitorObject)

bool SimulinkConstraintsConverter::create(const yarp::os::Property& options)
{
   printf("created!\n");
   printf("I am attached to the %s\n",
          (options.find("sender_side").asBool()) ? "sender side" : "receiver side");
    m_buffer[0] = m_buffer[1] = -1;
    m_isInitialized = false;
   return true;
}
void SimulinkConstraintsConverter::destroy(void)
{
    printf("destroyed!\n");
}
bool SimulinkConstraintsConverter::setparam(const yarp::os::Property& params)
{
    return false;
}
bool SimulinkConstraintsConverter::getparam(yarp::os::Property& params)
{
    return false;
}
bool SimulinkConstraintsConverter::accept(yarp::os::Things& thing)
{
//    yarp::sig::Vector *data = thing.cast_as<yarp::sig::Vector>();
//    if (!data) {
//        printf("SimpleMonitorObject: expected type Bottle but got wrong data type!\n");
//        return false;
//    }
//    if (data->size() < 2) {
//        printf("SimpleMonitorObject: expected type Bottle but got wrong data type!\n");
//        return false;
//    }
//
//    if (!isFloatEqual(m_buffer[0],(*data)[0], 1e-2) || !isFloatEqual(m_buffer[1],(*data)[1], 1e-2)) {
//        //        m_buffer[0] = (*data)[0];
//        //        m_buffer[1] = (*data)[1];
//        return true;
//    }
//
//
    Bottle *data = thing.cast_as<Bottle>();
    if (!data) {
        printf("SimpleMonitorObject: expected type Bottle but got wrong data type!\n");
        return false;
    }
    if (data->size() < 2) {
        printf("SimpleMonitorObject: expected type Bottle but got wrong data type!\n");
        return false;
    }

    if (!isFloatEqual(m_buffer[0], data->get(0).asDouble(), 1e-2)
        || !isFloatEqual(m_buffer[1], data->get(1).asDouble(), 1e-2)) {
        return true;
    }

    return false;
}
yarp::os::Things& SimulinkConstraintsConverter::update(yarp::os::Things& thing)
{
    Bottle *data = thing.cast_as<Bottle>();
    if (!data) {
        printf("SimpleMonitorObject: expected type Bottle but got wrong data type!\n");
        return thing;
    }
    double currentValue0 = data->get(0).asDouble();
    double currentValue1 = data->get(1).asDouble();

    data->clear();
    //Handle initial condition
    if (!m_isInitialized) {
        m_isInitialized = true;
        if (isFloatEqual(currentValue0, 1, 1e-2)) {
            data->addString("activateConstraints l_sole");
        } else if (isFloatEqual(currentValue0, 0, 1e-2)) {
            data->addString("deactivateConstraints l_sole");
        }

        if (isFloatEqual(currentValue1, 1, 1e-2)) {
            data->addString("activateConstraints r_sole");
        } else if (isFloatEqual(currentValue1, 0, 1e-2)) {
            data->addString("deactivateConstraints r_sole");
        }
        m_buffer[0] = currentValue0;
        m_buffer[1] = currentValue1;
        return thing;
    }

    if (!isFloatEqual(m_buffer[0], currentValue0, 1e-2)) {
        if (isFloatEqual(m_buffer[0], 0, 1e-2) && isFloatEqual(1, currentValue0, 1e-2)) {
            data->addString("activateConstraints l_sole");
        } else if (isFloatEqual(m_buffer[0], 1, 1e-2) && isFloatEqual(0, currentValue0, 1e-2)) {
            data->addString("deactivateConstraints l_sole");
        } else {
            yWarning("ConstraintsMonitor. (old-new) %ld - %ld not allowed", m_buffer[0], currentValue0);
        }
        m_buffer[0] = currentValue0;
    }
    if (!isFloatEqual(m_buffer[1], currentValue1, 1e-2)) {
        if (isFloatEqual(m_buffer[1], 0, 1e-2) && isFloatEqual(1, currentValue1, 1e-2)) {
            data->addString("activateConstraints r_sole");
        } else if (isFloatEqual(m_buffer[1], 1, 1e-2) && isFloatEqual(0, currentValue1, 1e-2)) {
            data->addString("deactivateConstraints r_sole");
        } else {
            yWarning("ConstraintsMonitor. (old-new) %ld - %ld not allowed", m_buffer[1], currentValue1);
        }
        m_buffer[1] = currentValue1;
    }

    return thing;
}

static bool isFloatEqual(double a, double b, double tol) {
    return std::abs(a - b) < tol;
}
