/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia - Italian Institute of Technology
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

#ifndef ICUB_WALKING_IK_THREAD_H
#define ICUB_WALKING_IK_THREAD_H

#include <yarp/os/RateThread.h>
#include <yarp/os/Property.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <yarpWholeBodyInterface/yarpWholeBodyModel.h>

class iCubWalkingIKThread: public yarp::os::RateThread {
private:

public:
    iCubWalkingIKThread (int period);
    bool threadInit();
    void run();
    void threadRelease();
};

#endif
