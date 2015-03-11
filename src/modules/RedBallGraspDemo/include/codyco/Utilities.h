/*
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * website: www.robotcub.org
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

#ifndef UTILITIES_H
#define UTILITIES_H

#include <yarp/os/Searchable.h>
#include <yarp/dev/all.h>
#include <yarp/sig/Vector.h>
#include <iCub/ctrl/neuralNetworks.h>

namespace codyco {

    class Predictor
    {
    protected:
        iCub::ctrl::ff2LayNN_tansig_purelin net;

    public:
        bool configure(yarp::os::Property &options);

        yarp::sig::Vector predict(const yarp::sig::Vector &head, yarp::os::Bottle *imdLeft, yarp::os::Bottle *imdRight);
    };


    class myReport : public yarp::os::SearchMonitor
    {
    protected:
        yarp::os::Property comment, fallback, present, actual, reported;
        yarp::os::Bottle order;

    public:
        void report(const yarp::os::SearchReport& report, const char *context);
    };
    
}
#endif /* end of include guard: UTILITIES_H */
