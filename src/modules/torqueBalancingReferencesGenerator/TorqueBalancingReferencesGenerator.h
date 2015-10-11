/**
 * Copyright (C) 2015 CoDyCo
 * @author: Daniele Pucci
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

#ifndef MODULE_H
#define MODULE_H

#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/LU>
#include <vector>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <stdio.h>



namespace wbi {
    class wholeBodyInterface;
}

class TorqueBalancingReferencesGenerator : public yarp::os::RFModule
{
    
    struct ComTimeAndSetPoints
    {
    yarp::sig::Vector    comDes;
    double    time;

    void reset()
    {
        comDes.resize(3,0);
        time = 0;
    }
    };
    
    struct Postures
    {
    yarp::sig::Vector    qDes;
    double    time;

    void reset(int NDOF)
    {
        qDes.resize(NDOF,0);
        time = 0;
    }
    };
private:
    wbi::wholeBodyInterface* m_robot;
    yarp::sig::Vector qDes;
    yarp::sig::Vector q0;
    yarp::sig::Vector q;

    yarp::sig::Vector   comDes;
    yarp::sig::Vector  DcomDes;
    yarp::sig::Vector DDcomDes;
    yarp::sig::Vector com0;

    yarp::sig::Vector directionOfOscillation;
    double            amplitudeOfOscillation;
    double            frequencyOfOscillation;    
    double            t0;
    double            period;
    
    std::vector<Postures> postures;
    
    std::vector<ComTimeAndSetPoints> comTimeAndSetPoints;
    
    yarp::os::BufferedPort<yarp::sig::Vector> portForStreamingComDes;
    yarp::os::BufferedPort<yarp::sig::Vector> portForStreamingQdes;;
    
    double timeoutBeforeStreamingRefs;
    
    bool   changePostural;
    bool   changeComWithSetPoints;
    
    std::string robotName;
//     int numberOfPostures;
    int centerOfMassLinkID;

    bool loadReferences(yarp::os::Searchable& config,
                        std::vector<Postures> & Postures, std::vector<ComTimeAndSetPoints> & ComTimeAndSetPoints,
                        double actuatedDOFs, bool & changePostural, bool & changeComWithSetPoints, 
                        double & amplitudeOfOscillation, 
                        double & frequencyOfOscillation,yarp::sig::Vector & directionOfOscillation  ); 
public:
    virtual double getPeriod ();
    virtual bool  updateModule ();
    virtual bool  configure (yarp::os::ResourceFinder &rf);
    virtual bool  close ();
};


#endif /* end of include guard: MODULE_H */
