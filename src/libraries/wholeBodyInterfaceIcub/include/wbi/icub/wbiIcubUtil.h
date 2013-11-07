/*
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Andrea Del Prete
 * email: andrea.delprete@iit.it
 *
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

#ifndef WBI_ICUB_UTIL_H
#define WBI_ICUB_UTIL_H

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <iCub/skinDynLib/common.h>
#include <wbi/wbi.h>
#include <vector>

/* CODE UNDER DEVELOPMENT */

namespace wbiy
{
    // handy variable for initializing the whole body interface for iCub
    const wbi::LocalIdList ICUB_RIGHT_ARM_JOINTS(iCub::skinDynLib::RIGHT_ARM, 0, 1, 2, 3, 4);
    const wbi::LocalIdList ICUB_LEFT_ARM_JOINTS(iCub::skinDynLib::LEFT_ARM, 0, 1, 2, 3, 4);
    const wbi::LocalIdList ICUB_RIGHT_LEG_JOINTS(iCub::skinDynLib::RIGHT_LEG, 0, 1, 2, 3, 4, 5);
    const wbi::LocalIdList ICUB_LEFT_LEG_JOINTS(iCub::skinDynLib::LEFT_LEG, 0, 1, 2, 3, 4, 5);
    const wbi::LocalIdList ICUB_TORSO_JOINTS(iCub::skinDynLib::TORSO, 0, 1, 2);
    const wbi::LocalIdList ICUB_MAIN_JOINTS(ICUB_TORSO_JOINTS, ICUB_LEFT_ARM_JOINTS, ICUB_RIGHT_ARM_JOINTS, ICUB_LEFT_LEG_JOINTS, ICUB_RIGHT_LEG_JOINTS);

    // mapping from generic sensor id to corresponding port name
    struct id_2_PortName { wbi::LocalId id; std::string portName; };

    // *** Mapping from FORCE/TORQUE SENSORS to PORT NAMES ***
    const int ICUB_FT_SENS_NUMBER = 6;
    const id_2_PortName icub_FTsens_2_PortName[ICUB_FT_SENS_NUMBER] = {
        {wbi::LocalId(iCub::skinDynLib::LEFT_ARM,0),    "/left_arm/analog:o" }, 
        {wbi::LocalId(iCub::skinDynLib::RIGHT_ARM,0),   "/right_arm/analog:o"}, 
        {wbi::LocalId(iCub::skinDynLib::LEFT_LEG,0),    "/left_leg/analog:o" }, 
        {wbi::LocalId(iCub::skinDynLib::RIGHT_LEG,0),   "/right_leg/analog:o"}, 
        {wbi::LocalId(iCub::skinDynLib::LEFT_LEG,1),    "/left_foot/analog:o"}, 
        {wbi::LocalId(iCub::skinDynLib::RIGHT_LEG,1),   "/right_foot/analog:o"}, 
    };

    // *** Mapping from IMUs to PORT NAMES ***
    const id_2_PortName icub_IMU_2_PortName[1] = {
        {wbi::LocalId(iCub::skinDynLib::HEAD,0),    "/inertial:o" }, 
    };

    /** Find the port name into id2port corresponding to the specified local id.
     * @param lid Local id to look for
     * @param id2port Mapping from ids to port names
     * @param size Number of elements of id2port
     * @return The port name corresponding to the specified local id. */
    inline std::string getPortName(const wbi::LocalId &lid, const id_2_PortName *id2port, const int size)
    {
        int i=0;
        do
        {
            if(id2port[i].id == lid)
                return id2port[i].portName;
            i++;
        }
        while(i<size);
        return "";
    }

    inline std::string getPortName(const wbi::LocalId &lid, const std::vector<id_2_PortName> id2port)
    {return getPortName(lid, &id2port[0], id2port.size());}

    /** Return true if the robotName is "icubSim", false otherwise. */
    inline bool isRobotSimulator(const std::string &robotName)
    { return robotName=="icubSim"; }
    
    /** Open a remote control board driver for the specified body part. */
    inline bool openPolyDriver(const std::string &localName, const std::string &robotName, yarp::dev::PolyDriver *&pd, const std::string &bodyPartName)
    {
        std::string localPort  = "/" + localName + "/" + bodyPartName;
        std::string remotePort = "/" + robotName + "/" + bodyPartName;
        yarp::os::Property options;
        options.put("robot",robotName.c_str());
        options.put("part",bodyPartName.c_str());
        options.put("device","remote_controlboard");
        options.put("local",localPort.c_str());
        options.put("remote",remotePort.c_str());
    
        pd = new yarp::dev::PolyDriver(options);
        if(!pd || !(pd->isValid()))
        {
            std::fprintf(stderr,"Problems instantiating the device driver %s\n", bodyPartName.c_str());
            return false;
        }
        return true;
    }
    
} // end namespace

#endif
