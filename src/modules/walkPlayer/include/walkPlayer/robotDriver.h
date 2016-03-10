#ifndef ROBOTDRIVER_H
#define ROBOTDRIVER_H

#include <yarp/os/Property.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IInteractionMode.h>
#include <yarp/math/Math.h>
#include <yarp/sig/Matrix.h>
#include <iCub/ctrl/math.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include "actionClass.h"


// ******************** ROBOT DRIVER CLASS
class robotDriver
{
public:
    bool verbose;
    bool connected;
    yarp::os::Property          drvOptions_ll;
    yarp::os::Property          drvOptions_rl;
    yarp::os::Property          drvOptions_to;
    yarp::dev::PolyDriver       *drv_ll;
    yarp::dev::PolyDriver       *drv_rl;
    yarp::dev::PolyDriver       *drv_to;
    yarp::dev::IPositionControl *ipos_ll;
    yarp::dev::IPidControl      *ipid_ll;
    yarp::dev::IImpedanceControl *iimp_ll;
    yarp::dev::IInteractionMode *iint_ll;
    yarp::dev::IEncoders        *ienc_ll;
    yarp::dev::IPositionControl *ipos_rl;
    yarp::dev::IPidControl      *ipid_rl;
    yarp::dev::IImpedanceControl *iimp_rl;
    yarp::dev::IInteractionMode *iint_rl;
    yarp::dev::IEncoders        *ienc_rl;
    yarp::dev::IPositionControl *ipos_to;
    yarp::dev::IPidControl      *ipid_to;
    yarp::dev::IEncoders        *ienc_to;
    yarp::dev::IControlMode2    *icmd_ll;
    yarp::dev::IControlMode2    *icmd_rl;
    yarp::dev::IControlMode2    *icmd_to;
    yarp::dev::IPositionDirect  *idir_rl;
    yarp::dev::IPositionDirect  *idir_ll;
    yarp::dev::IPositionDirect  *idir_to;
    iCub::iDyn::iCubWholeBody   *icub_dyn;

public:
    robotDriver();
    yarp::sig::Matrix compute_transformations (actionStruct act);
    bool configure(const yarp::os::Property &copt);
    bool init();
    ~robotDriver();
};


#endif
