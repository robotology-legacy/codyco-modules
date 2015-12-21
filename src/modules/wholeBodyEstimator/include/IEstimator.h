/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia - Italian Institute of Technology
 * Author: Jorhabib Eljaik
 * email:  jorhabib.eljaik@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef IESTIMATOR_H_
#define IESTIMATOR_H_

#include <yarp/os/ResourceFinder.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
// We need to include the factory here so that the derived classes of IEstimator can use the macros defined there.
#include "EstimatorsFactory.h"

class IEstimator
{
public:
    /**
     *  Virtual destructor which is implemented in IDestructors.cpp
     *
     */
    virtual ~IEstimator() = 0;
    /**
     Initializes a generic estimator. Use this method to initialize variables used by your estimator. You could also create here your system and measurement model, as well as the filter used for estimation. Ports used by the estimator (if needed) should also be opened here and configured here. 
     
     - parameter yarp: rf is a ResourceFinder object reference that should contain all the information read from configuration file.
     - parameter wbi:  Pointer to a wholeBodySensors object that must have been initialized and configured beforehand.
     
     - returns: true when initialization is successful. False otherwise.
     */
    virtual bool init(yarp::os::ResourceFinder &rf, wbi::iWholeBodySensors *wbs) = 0;
    /**
     *  Runs main estimation loop in the implementation.
     */
    virtual void run() = 0;
    /**
     *  Releases allocated resources and closes opened ports during initialization.
     */
    virtual void release() = 0;
};

#endif
