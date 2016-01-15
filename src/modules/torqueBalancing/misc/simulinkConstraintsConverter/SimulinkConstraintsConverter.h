#ifndef SIMULINKCONSTRAINTSCONVERTER_H
#define SIMULINKCONSTRAINTSCONVERTER_H

#include <yarp/os/MonitorObject.h>

namespace codyco {
    namespace torquebalancing {
        class SimulinkConstraintsConverter;
    }
}

class codyco::torquebalancing::SimulinkConstraintsConverter : public yarp::os::MonitorObject
{

    double m_buffer[2];
    bool m_isInitialized;
public:
    bool create(const yarp::os::Property& options);
    void destroy(void);
    bool setparam(const yarp::os::Property& params);
    bool getparam(yarp::os::Property& params);
    bool accept(yarp::os::Things& thing);
    yarp::os::Things& update(yarp::os::Things& thing);
};

#endif /* end of include guard: SIMULINKCONSTRAINTSCONVERTER_H */
