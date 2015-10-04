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

#include <TorqueBalancingReferencesGenerator.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <wbi/wholeBodyInterface.h>
#include <wbi/wbiUtil.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Property.h>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <stdio.h>

#include <cmath>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <Eigen/LU>

#include <vector>

double TorqueBalancingReferencesGenerator::getPeriod () { return period; }

bool TorqueBalancingReferencesGenerator::updateModule ()
{
    using namespace yarp::math;
    double    t = yarp::os::Time::now();
    if (t-t0 < timeoutBeforeStreamingRefs)
    {    
        comDes = com0 ;
        qDes = q0;
        
        
    }
    else
    {
        comDes = com0 + amplitudeOfOscillation*sin(2*M_PI*frequencyOfOscillation*(t-t0-timeoutBeforeStreamingRefs))*directionOfOscillation;
        DcomDes =  2*M_PI*frequencyOfOscillation*amplitudeOfOscillation*cos(2*M_PI*frequencyOfOscillation*(t-t0-timeoutBeforeStreamingRefs))*directionOfOscillation;
        DDcomDes = -2*M_PI*frequencyOfOscillation*2*M_PI*frequencyOfOscillation*amplitudeOfOscillation*sin(2*M_PI*frequencyOfOscillation*(t-t0-timeoutBeforeStreamingRefs))*directionOfOscillation;
    
        if (changePostural)
        {
            qDes = q0;
            for (int i = 0 ; i < postures.size()-1; i++)
            {
                if ((t-t0) > postures[i].time && (t-t0) <= postures[i+1].time )
                {
                    qDes = postures[i].qDes;
                    break;
                }
                qDes = postures[postures.size()-1].qDes;
            }
        }
    }
    
    yarp::sig::Vector& output = portForStreamingComDes.prepare();
    output.resize(9);
    output.setSubvector(0,comDes);
    output.setSubvector(3,DcomDes);
    output.setSubvector(6,DDcomDes);
    portForStreamingComDes.write();

    yarp::sig::Vector& output2 = portForStreamingQdes.prepare();
    output2 = qDes;
    portForStreamingQdes.write();
    
    return true;
}

bool TorqueBalancingReferencesGenerator::configure (yarp::os::ResourceFinder &rf)
{
    using namespace yarp::os;
    using namespace yarp::sig;
    using namespace Eigen;

    
    if (!rf.check("wbi_config_file", "Checking wbi configuration file")) {
        std::cout << "No WBI configuration file found.\n";
        return false;
    }
    
    Property wbiProperties;
    if (!wbiProperties.fromConfigFile(rf.findFile("wbi_config_file"))) {
        std::cout << "Not possible to load WBI properties from file.\n";
        return false;
    }
    wbiProperties.fromString(rf.toString(), false);

    //retrieve the joint list
    std::string wbiList = rf.find("wbi_joint_list").asString();

    wbi::IDList iCubMainJoints;
    if (!yarpWbi::loadIdListFromConfig(wbiList, wbiProperties, iCubMainJoints)) {
        std::cout << "Cannot find joint list\n";
        return false;
    }

    double actuatedDOFs = iCubMainJoints.size();

    //create an instance of wbi
    m_robot = new yarpWbi::yarpWholeBodyInterface("torqueBalancingReferencesGenerator", wbiProperties);
    if (!m_robot) {
        std::cout << "Could not create wbi object.\n";
        return false;
    }

    m_robot->addJoints(iCubMainJoints);
    if (!m_robot->init()) {
        std::cout << "Could not initialize wbi object.\n";
        return false;
    }
    
    robotName = rf.check("robot", Value("icubGazeboSim"), "Looking for robot name").asString();
//     numberOfPostures = rf.check("numberOfPostures", Value(0), "Looking for numberOfPostures").asInt();

    directionOfOscillation.resize(3,0.0);
    frequencyOfOscillation = 0;
    amplitudeOfOscillation = 0;
    if (!loadReferences(rf,postures,actuatedDOFs,changePostural, amplitudeOfOscillation,frequencyOfOscillation, directionOfOscillation  ))
    {
        return false;
    }
    loadReferences(rf,postures,actuatedDOFs,changePostural, amplitudeOfOscillation,frequencyOfOscillation, directionOfOscillation  );
    for(int i=0; i < postures.size(); i++ )
    {
        std::cerr << "[INFO] time_" << i << " = " << postures[i].time << std::endl;
    }
    for(int i=0; i < postures.size(); i++ )
    {
        std::cerr << "[INFO] posture_" << i << " = " << postures[i].qDes.toString()<< std::endl;
        
    }
//     
    std::cout << "[INFO] Number of DOFs: " << actuatedDOFs << std::endl;
    
    std::cerr << "[INFO] changePostural: " << changePostural << std::endl;
    
    std::cerr << "[INFO] amplitudeOfOscillation: " << amplitudeOfOscillation << std::endl;

    std::cout << "[INFO] frequencyOfOscillation " << frequencyOfOscillation << std::endl;
    
    std::cerr << "[INFO] directionOfOscillation: " << directionOfOscillation.toString() << std::endl;

    q0.resize(actuatedDOFs, 0.0);
    com0.resize(3, 0.0);
    comDes.resize(3, 0.0);
    DcomDes.resize(3, 0.0);
    DDcomDes.resize(3, 0.0);
    m_robot->getEstimates(wbi::ESTIMATE_JOINT_POS, q0.data());
    
    double world2BaseFrameSerialization[16];
    double rotoTranslationVector[7];
    wbi::Frame world2BaseFrame;
    m_robot->getEstimates(wbi::ESTIMATE_BASE_POS, world2BaseFrameSerialization);
    wbi::frameFromSerialization(world2BaseFrameSerialization, world2BaseFrame);
    m_robot->forwardKinematics(q0.data(), world2BaseFrame, wbi::wholeBodyInterface::COM_LINK_ID, rotoTranslationVector);
    com0[0] = rotoTranslationVector[0];    
    com0[1] = rotoTranslationVector[1];
    com0[2] = rotoTranslationVector[2];
    
    timeoutBeforeStreamingRefs = rf.check("timeoutBeforeStreamingRefs", Value(20), "Looking for robot name").asDouble();

    period = rf.check("period", Value(0.01), "Looking for module period").asDouble();
    
    std::cout << "timeoutBeforeStreamingRefs: " << timeoutBeforeStreamingRefs << "\n";

    std::string group_name = "PORTS_INFO";
    if( !rf.check(group_name) )
    {
        std::cerr << "[ERR] no PORTS_INFO group found" << std::endl;
        return true;
    }
    else
    {
        yarp::os::Bottle group_bot = rf.findGroup(group_name);

        if( !group_bot.check("portNameForStreamingQdes") || 
            !group_bot.check("portNameForStreamingComDes") )
        {
             std::cerr << "[ERR] portNameForStreamingQdes or portNameForStreamingComDes not found in config file" << std::endl;
                    return false;
        }

        //Check coupling configuration is well formed
        std::string portNameForStreamingComDes = group_bot.find("portNameForStreamingComDes").asString();
        std::cout << "portNameForStreamingComDes: " << portNameForStreamingComDes << "\n";
        
        std::string portNameForStreamingQdes = group_bot.find("portNameForStreamingQdes").asString();
        std::cout << "portNameForStreamingQdes: " << portNameForStreamingQdes << "\n";

        portForStreamingComDes.open(portNameForStreamingComDes);
        
        portForStreamingQdes.open(portNameForStreamingQdes);
        
        t0 = yarp::os::Time::now();
    return true;
    }
}

bool TorqueBalancingReferencesGenerator::close ()
{
    //cleanup stuff
    m_robot->close();
    delete m_robot;
    m_robot = 0;
    return true;
}



bool TorqueBalancingReferencesGenerator::loadReferences(yarp::os::Searchable& config,
                                            std::vector<Postures> & postures, double actuatedDOFs, bool & changePostural, double & amplitudeOfOscillation, double & frequencyOfOscillation,yarp::sig::Vector & directionOfOscillation  )
{
    std::string group_name = "REFERENCES";
    if( !config.check(group_name) )
    {
        std::cerr << "[ERR] no REFERENCES group found" << std::endl;
        return true;
    }
    else
    {
        yarp::os::Bottle group_bot = config.findGroup(group_name);

        if( !group_bot.check("postures") )
        {
             std::cerr << "[INFO] postures found in config file, qDes kept at initial condition" << std::endl;
             changePostural = false;
        }
        else
        {
            if( group_bot.check("posturesInRadians") &&
                group_bot.check("posturesInDegrees"))
            {
                std::cerr << "[ERR] Both posturesInRadians and posturesInDegrees found in the configuration file. Incapable of interpreting postures' unit of measurement" << std::endl;
                return false;
            }
            
            if( !(group_bot.check("posturesInRadians") ||
                  group_bot.check("posturesInDegrees")))
            {
                std::cerr << "[ERR] Neither posturesInRadians nor posturesInDegrees found in the configuration file. Incapable of interpreting postures' unit of measurement" << std::endl;
                return false;
            }
            
            double conversion = 1;
            bool posturesInRadians = true;
            if(group_bot.check("posturesInDegrees"))
            {
                conversion = M_PI/180;
                posturesInRadians = false;
            }
            
            changePostural = true;
            yarp::os::Bottle * postures_bot = group_bot.find("postures").asList();

            int numberOfPostures = postures_bot->size();
            postures.resize(numberOfPostures);
            Postures post;
            post.reset(actuatedDOFs);
            for(int i=0; i < numberOfPostures; i++ )
            {
                if( !(postures_bot->get(i).isList()) ||
                    !(postures_bot->get(i).asList()->size() == actuatedDOFs + 1) )
                {
                    std::cerr << "[ERR] " << group_name << " group found, but is of wrong dimension. Recall that if you are controlling NDOFs, the variable postures must contain NDOFs+1 element "
                    << std::endl;
                    return false;
                }
                yarp::os::Bottle * postures_bot_i = postures_bot->get(i).asList();
                post.time = postures_bot_i->get(0).asDouble();
                if (i > 0)
                {
                    if (postures_bot_i->get(0).asDouble() <= postures[i-1].time )
                    {
                        std::cerr << "[ERR] Time series of postures is not strictly increasing"  << std::endl;
                        return false;
                    }
                }
                for (int j=0; j < actuatedDOFs; j++)
                {
                    post.qDes[j] = conversion*(postures_bot_i->get(j+1).asDouble());
                    if (posturesInRadians)
                    {
                        if ( (post.qDes[j] > 2*M_PI) || (post.qDes[j] < -2*M_PI) )
                        {
                            std::cerr << "[ERR] postures are expected to be in radians, but they exceed the limits [-2*pi,2*pi]"<< std::endl;
                            return false;
                        }
                    }
                }
                
                postures[i] = post;
                
            }
        }
        if(group_bot.check("directionOfOscillation") || group_bot.check("amplitudeInMeters") || group_bot.check("frequencyInHerz"))
        {
            if( !(group_bot.check("directionOfOscillation")) ||
                !(group_bot.check("amplitudeInMeters")) ||
                !(group_bot.check("frequencyInHerz")))
            {
                std::cerr << "[ERR] directionOfOscillation or amplitudeInMeters or frequencyInHerz is missing" << std::endl;
                return false;
            }
            
            if( !(group_bot.find("directionOfOscillation").isList()) ||
                !(group_bot.find("directionOfOscillation").asList()->size() == 3))
            {
                std::cerr << "[ERR] directionOfOscillation is malformed" << std::endl;
                return false;
            }

            //Check coupling configuration is well formed

            double normOfDirectionOfOscillation = 0;
            for(int i=0; i < 3; i++ )
            {
                directionOfOscillation[i] = group_bot.find("directionOfOscillation").asList()->get(i).asDouble();            
                normOfDirectionOfOscillation += directionOfOscillation[i]*directionOfOscillation[i];
            }
            if( (normOfDirectionOfOscillation < 0.99) || (normOfDirectionOfOscillation > 1.01)) 
            {
                std::cerr << "[ERR] directionOfOscillation is not of unit norm" << std::endl;
                return false;
            }
            amplitudeOfOscillation = group_bot.check("amplitudeInMeters", yarp::os::Value(0), "Looking for amplitudeOfOscillation").asDouble();
                    
            frequencyOfOscillation = group_bot.check("frequencyInHerz", yarp::os::Value(0), "Looking for frequencyOfOscillation").asDouble();
      
        }
      
        return true;
    }

}
