/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#ifndef IDYN_KDL_EMULATION_H
#define IDYN_KDL_EMULATION_H

#include <iCub/iDyn/iDyn.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/api.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/skinDynLib/skinContactList.h>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

#include "../iDyn_KDL_conversion/iDyn2KDL.h"
#include "../iDyn_KDL_conversion/KDL2iDyn.h"

#include <cassert>

#include "custom_kdl/chainidsolver_recursive_newton_euler_internal_wrenches.hpp"

using namespace yarp::math;


//for now, for this methods, assuming dw0,w0,ddp0 = 0.0, q,dq,ddq already setted in the chain, and with kinematic information (position) updated based on q
//And no joint blocked!
//Same functions, but no more assuming ddp0 = 0.0
//if ddp0 = 0.0, assuming H0 = eye()
/**
 * Same as iCub::iDyn::iDynChain::getForces() method, but using KDL for calculation
 */
yarp::sig::Matrix idynChainGetForces_usingKDL(iCub::iDyn::iDynChain & idynChain,yarp::sig::Vector & ddp0);

/**
 * Same as iCub::iDyn::iDynChain::getMoments() method, but using KDL for calculation
 */
yarp::sig::Matrix idynChainGetMoments_usingKDL(iCub::iDyn::iDynChain & idynChain,yarp::sig::Vector & ddp0);

/**
 * Same as iCub::iDyn::iDynChain::getTorques() method, but using KDL for calculation
 */
yarp::sig::Vector idynChainGetTorques_usingKDL(iCub::iDyn::iDynChain & idynChain,yarp::sig::Vector & ddp0);

/**
 * Function for internal use
 */
KDL::Wrenches idynChainGet_usingKDL_aux(iCub::iDyn::iDynChain & idynChain,yarp::sig::Vector & ddp0);


yarp::sig::Matrix idynChainGetForces_usingKDL(iCub::iDyn::iDynChain & idynChain,iCub::iDyn::iDynInvSensor & idynSensor,yarp::sig::Vector & ddp0);
yarp::sig::Matrix idynChainGetMoments_usingKDL(iCub::iDyn::iDynChain & idynChain,iCub::iDyn::iDynInvSensor & idynSensor,yarp::sig::Vector & ddp0);
yarp::sig::Vector idynChainGetTorques_usingKDL(iCub::iDyn::iDynChain & idynChain,iCub::iDyn::iDynInvSensor & idynSensor,yarp::sig::Vector & ddp0);

KDL::Wrenches idynChainGet_usingKDL_aux(iCub::iDyn::iDynChain & idynChain,iCub::iDyn::iDynInvSensor & idynSensor,yarp::sig::Vector & ddp0);


yarp::sig::Vector idynSensorGetSensorForce_usingKDL(iCub::iDyn::iDynChain & idynChain,iCub::iDyn::iDynInvSensor & idynSensor,yarp::sig::Vector & ddp0);
yarp::sig::Vector idynSensorGetSensorMoment_usingKDL(iCub::iDyn::iDynChain & idynChain,iCub::iDyn::iDynInvSensor & idynSensor,yarp::sig::Vector & ddp0);


#endif
