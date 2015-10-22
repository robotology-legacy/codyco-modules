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

#ifndef WHOLEBODYESTIMATORMODULE_H_
#define WHOLEBODYESTIMATORMODULE_H_

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Property.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include "WholeBodyEstimatorThread.h"


class WholeBodyEstimatorModule: public yarp::os::RFModule {
private:
    yarp::os::Property              m_module_params;
    double                          m_period;
    std::string                     m_module_name;
    WholeBodyEstimatorThread        *m_estimatorThread;
public:
    WholeBodyEstimatorModule();
    bool configure(yarp::os::ResourceFinder &rf);
    double getPeriod(){ return m_period;}
    bool updateModule();
    bool close();
};

#endif