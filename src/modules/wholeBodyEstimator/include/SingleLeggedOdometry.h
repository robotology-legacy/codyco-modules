/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia - Italian Institute of Technology
 * Authors: Silvio Traversaro, Jorhabib Eljaik
 * email: silvio(dot)traversaro(at)iit(dot)it, jorhabib(dot)eljaik(at)iit(dot)it
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

#include "IEstimator.h"

#ifndef _SINGLEODOMETRY_H_
#define _SINGLEODOMETRY_H_

class SingleLeggedOdometry : public IEstimator
{
    bool init(yarp::os::ResourceFinder &rf);
    void run();
    void release();
    
};

#endif /* SingleLeggedOdometry */
