/*
 * Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia - Italian Institute of Technology
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

#ifndef __QUATERNIONEKFMODULE_H__
#define __QUATERNIONEKFMODULE_H__

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/RFModule.h>
#include <yarp/math/Math.h>
#include <iostream>
#include <iostream>
#include <fstream>
#include "quaternionEKFThread.h"
#define FILTER_GROUP_PARAMS_NAME "EKFPARAMS"
#define DIRECT_GROUP_PARAMS_NAME "DIRECTFILTERPARAMS"
#define CONVERSION_FACTOR_ACC 5.9855e-04

namespace filter{
class quaternionEKFModule: public yarp::os::RFModule
{
    double                                      period;
    std::string                                 robotName;
    std::string                                 local;
    std::string                                 sensorPortName;
    bool                                        autoconnect;
    bool                                        usingEKF;
    bool                                        usingSkin;
    bool                                        inWorldRefFrame;
    bool                                        debugGyro;
    bool                                        debugAcc;
    bool                                        calib;
    bool                                        using2acc;
    std::string                                 mode;
    bool                                        usingxsens;
    bool                                        verbose;
    
    /*TODO : For now filtertype is a string to indicate EKF or direct filtering, 
     *later when module name is more generic it must be changed to enum */
    std::string                                 filterType; 
    
    yarp::os::BufferedPort<yarp::sig::Vector>   gyroMeasPort;
    yarp::os::BufferedPort<yarp::sig::Vector>   gyroMeasPort2;
    quaternionEKFThread                        *quatEKFThread;
    
    // Parser parameters
    dataDumperParser                           *m_parser;
    currentData                                 m_currentData;
    
public:
    quaternionEKFModule();
    
    bool   configure(yarp::os::ResourceFinder &rf);
    bool   close();
    double getPeriod(){ return period; }
    bool   updateModule();
};

}

#endif


