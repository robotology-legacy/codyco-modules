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
#include <cstdio>

/* CODE UNDER DEVELOPMENT */

namespace wbiIcub
{
    ///< handy variables for initializing the whole body interface for iCub
    const wbi::LocalIdList ICUB_TORSO_JOINTS(iCub::skinDynLib::TORSO, 0, 1, 2);
    const wbi::LocalIdList ICUB_LEFT_ARM_JOINTS(iCub::skinDynLib::LEFT_ARM, 0, 1, 2, 3, 4);
    const wbi::LocalIdList ICUB_RIGHT_ARM_JOINTS(iCub::skinDynLib::RIGHT_ARM, 0, 1, 2, 3, 4);
    const wbi::LocalIdList ICUB_LEFT_LEG_JOINTS(iCub::skinDynLib::LEFT_LEG, 0, 1, 2, 3, 4, 5);
    const wbi::LocalIdList ICUB_RIGHT_LEG_JOINTS(iCub::skinDynLib::RIGHT_LEG, 0, 1, 2, 3, 4, 5);
    const wbi::LocalIdList ICUB_MAIN_JOINTS(ICUB_TORSO_JOINTS, ICUB_LEFT_ARM_JOINTS, ICUB_RIGHT_ARM_JOINTS, ICUB_LEFT_LEG_JOINTS, ICUB_RIGHT_LEG_JOINTS);
    ///< Joints considered for dynamical model
    const wbi::LocalIdList ICUB_HEAD_JOINTS(iCub::skinDynLib::HEAD, 0, 1, 2);
    const wbi::LocalIdList ICUB_LEFT_ARM_DYNAMIC_JOINTS(iCub::skinDynLib::LEFT_ARM, 0, 1, 2, 3, 4, 5, 6);
    const wbi::LocalIdList ICUB_RIGHT_ARM_DYNAMIC_JOINTS(iCub::skinDynLib::RIGHT_ARM, 0, 1, 2, 3, 5, 6);
    const wbi::LocalIdList ICUB_MAIN_DYNAMIC_JOINTS(ICUB_TORSO_JOINTS, ICUB_HEAD_JOINTS, ICUB_LEFT_ARM_DYNAMIC_JOINTS, ICUB_RIGHT_ARM_DYNAMIC_JOINTS, ICUB_LEFT_LEG_JOINTS, ICUB_RIGHT_LEG_JOINTS);
    
    ///< mapping from generic sensor id to corresponding port name
    struct id_2_PortName { wbi::LocalId id; std::string portName; };

    ///< Mapping from FORCE/TORQUE SENSORS to PORT NAMES ***
    const int ICUB_FT_SENS_NUMBER = 6;
    const id_2_PortName icub_FTsens_2_PortName[ICUB_FT_SENS_NUMBER] = {
        {wbi::LocalId(iCub::skinDynLib::LEFT_ARM,0),    "/left_arm/analog:o" }, 
        {wbi::LocalId(iCub::skinDynLib::RIGHT_ARM,0),   "/right_arm/analog:o"}, 
        {wbi::LocalId(iCub::skinDynLib::LEFT_LEG,0),    "/left_leg/analog:o" }, 
        {wbi::LocalId(iCub::skinDynLib::RIGHT_LEG,0),   "/right_leg/analog:o"}, 
        {wbi::LocalId(iCub::skinDynLib::LEFT_LEG,1),    "/left_foot/analog:o"}, 
        {wbi::LocalId(iCub::skinDynLib::RIGHT_LEG,1),   "/right_foot/analog:o"}, 
    };
    
    ///< List of FT sensors in iCub
    const wbi::LocalIdList ICUB_LEFT_ARM_FTS(iCub::skinDynLib::LEFT_ARM, 0);
    const wbi::LocalIdList ICUB_RIGHT_ARM_FTS(iCub::skinDynLib::RIGHT_ARM, 0);
    const wbi::LocalIdList ICUB_LEFT_LEG_FTS(iCub::skinDynLib::LEFT_LEG, 0);
    const wbi::LocalIdList ICUB_RIGHT_LEG_FTS(iCub::skinDynLib::RIGHT_LEG, 0);
    const wbi::LocalIdList ICUB_LEFT_LEG_FOOT_FTS(iCub::skinDynLib::LEFT_LEG, 0, 1);
    const wbi::LocalIdList ICUB_RIGHT_LEG_FOOT_FTS(iCub::skinDynLib::RIGHT_LEG, 0, 1);
    
    const wbi::LocalIdList ICUB_MAIN_FTS(ICUB_LEFT_ARM_FTS,ICUB_RIGHT_ARM_FTS,ICUB_LEFT_LEG_FTS,ICUB_RIGHT_LEG_FTS);
    const wbi::LocalIdList ICUB_MAIN_FOOT_FTS(ICUB_LEFT_ARM_FTS,ICUB_RIGHT_ARM_FTS,ICUB_LEFT_LEG_FTS,ICUB_LEFT_LEG_FOOT_FTS,ICUB_RIGHT_LEG_FTS,ICUB_RIGHT_LEG_FOOT_FTS);

    
    ///< List of IMUS in iCub
    const wbi::LocalIdList ICUB_HEAD_IMUS(iCub::skinDynLib::HEAD, 0);
    const wbi::LocalIdList ICUB_MAIN_IMUS(ICUB_HEAD_IMUS);

    
    ///< *** Mapping from IMUs to PORT NAMES ***
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

    /** Given a vectir if id_2_PortName and a LocalId, find the port name associated to the
     * specified LocalId.
     * @param lid The local id.
     * @param id2port The mapping from local ids to port names.
     * @return The port name associated to the specified local id. If nothing is found, it returns an empty string. */
    inline std::string getPortName(const wbi::LocalId &lid, const std::vector<id_2_PortName> id2port)
    {return getPortName(lid, &id2port[0], id2port.size());}

    /** Return true if the robotName is "icubSim", false otherwise. */
    inline bool isRobotSimulator(const std::string &robotName)
    { return robotName=="icubSim" || robotName=="icubGazeboSim"; }
    
    /** Open a remote control board driver for the specified body part. 
     * @param localName Name to use as stem for the names of the YARP ports to open.
     * @param robotName Name of the robot to connect to.
     * @param pd Pointer to the poly driver to instanciate. 
     * @param bodyPartName Name of the body part for which to open the poly driver.
     * @return True if the operation succeeded, false otherwise. */
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

    /** Convert iCub's IDs from the iDynTree convention to the iCub WBI convention.
     * @param globalJointId Id of a joint according to the iDynTree serialization.
     * @return Id of the joint according to the iCub WBI serialization. */
    inline wbi::LocalId globalToLocalIcubId(int globalJointId)
    {
        switch(globalJointId)
        {
        case 0: return wbi::LocalId(iCub::skinDynLib::TORSO, 0, "torso_pitch");
        case 1: return wbi::LocalId(iCub::skinDynLib::TORSO, 1, "torso_roll");
        case 2: return wbi::LocalId(iCub::skinDynLib::TORSO, 2, "torso_yaw");
        case 3: return wbi::LocalId(iCub::skinDynLib::HEAD, 0, "neck_pitch");
        case 4: return wbi::LocalId(iCub::skinDynLib::HEAD, 1, "neck_roll");
        case 5: return wbi::LocalId(iCub::skinDynLib::HEAD, 2, "neck_yaw");
        case 6: return wbi::LocalId(iCub::skinDynLib::LEFT_ARM, 0, "l_shoulder_pitch");
        case 7: return wbi::LocalId(iCub::skinDynLib::LEFT_ARM, 1, "l_shoulder_roll");
        case 8: return wbi::LocalId(iCub::skinDynLib::LEFT_ARM, 2, "l_shoulder_yaw");
        case 9: return wbi::LocalId(iCub::skinDynLib::LEFT_ARM, 3, "l_elbow");
        case 10: return wbi::LocalId(iCub::skinDynLib::LEFT_ARM, 4, "l_wrist_prosup");
        case 11: return wbi::LocalId(iCub::skinDynLib::LEFT_ARM, 5, "l_wrist_pitch");
        case 12: return wbi::LocalId(iCub::skinDynLib::LEFT_ARM, 6, "l_wrist_yaw");
        case 13: return wbi::LocalId(iCub::skinDynLib::RIGHT_ARM, 0, "r_shoulder_pitch");
        case 14: return wbi::LocalId(iCub::skinDynLib::RIGHT_ARM, 1, "r_shoulder_roll");
        case 15: return wbi::LocalId(iCub::skinDynLib::RIGHT_ARM, 2, "r_shoulder_yaw");
        case 16: return wbi::LocalId(iCub::skinDynLib::RIGHT_ARM, 3, "r_elbow");
        case 17: return wbi::LocalId(iCub::skinDynLib::RIGHT_ARM, 4, "r_wrist_prosup");
        case 18: return wbi::LocalId(iCub::skinDynLib::RIGHT_ARM, 5, "r_wrist_pitch");
        case 19: return wbi::LocalId(iCub::skinDynLib::RIGHT_ARM, 6, "r_wrist_yaw");
        case 20: return wbi::LocalId(iCub::skinDynLib::LEFT_LEG, 0, "l_hip_pitch");
        case 21: return wbi::LocalId(iCub::skinDynLib::LEFT_LEG, 1, "l_hip_roll");
        case 22: return wbi::LocalId(iCub::skinDynLib::LEFT_LEG, 2, "l_hip_yaw");
        case 23: return wbi::LocalId(iCub::skinDynLib::LEFT_LEG, 3, "l_knee");
        case 24: return wbi::LocalId(iCub::skinDynLib::LEFT_LEG, 4, "l_ankle_pitch");
        case 25: return wbi::LocalId(iCub::skinDynLib::LEFT_LEG, 5, "l_ankle_roll");
        case 26: return wbi::LocalId(iCub::skinDynLib::RIGHT_LEG, 0, "r_hip_pitch");
        case 27: return wbi::LocalId(iCub::skinDynLib::RIGHT_LEG, 1, "r_hip_roll");
        case 28: return wbi::LocalId(iCub::skinDynLib::RIGHT_LEG, 2, "r_hip_yaw");
        case 29: return wbi::LocalId(iCub::skinDynLib::RIGHT_LEG, 3, "r_knee");
        case 30: return wbi::LocalId(iCub::skinDynLib::RIGHT_LEG, 4, "r_ankle_pitch");
        case 31: return wbi::LocalId(iCub::skinDynLib::RIGHT_LEG, 5, "r_ankle_roll");
        default: 
            printf("ERROR Unknown global joint id: %d\n", globalJointId);
            return wbi::LocalId();
        }
    }

    inline wbi::LocalId globalToLocalIcubId(std::string jointName)
    {
        if(jointName=="torso_pitch")        return wbi::LocalId(iCub::skinDynLib::TORSO, 0, "torso_pitch");
        if(jointName=="torso_roll")         return wbi::LocalId(iCub::skinDynLib::TORSO, 1, "torso_roll");
        if(jointName=="torso_yaw")          return wbi::LocalId(iCub::skinDynLib::TORSO, 2, "torso_yaw");
        if(jointName=="neck_pitch")         return wbi::LocalId(iCub::skinDynLib::HEAD, 0, "neck_pitch");
        if(jointName=="neck_roll")          return wbi::LocalId(iCub::skinDynLib::HEAD, 1, "neck_roll");
        if(jointName=="neck_yaw")           return wbi::LocalId(iCub::skinDynLib::HEAD, 2, "neck_yaw");
        if(jointName=="l_shoulder_pitch")   return wbi::LocalId(iCub::skinDynLib::LEFT_ARM, 0, "l_shoulder_pitch");
        if(jointName=="l_shoulder_roll")    return wbi::LocalId(iCub::skinDynLib::LEFT_ARM, 1, "l_shoulder_roll");
        if(jointName=="l_shoulder_yaw")     return wbi::LocalId(iCub::skinDynLib::LEFT_ARM, 2, "l_shoulder_yaw");
        if(jointName=="l_elbow")            return wbi::LocalId(iCub::skinDynLib::LEFT_ARM, 3, "l_elbow");
        if(jointName=="l_wrist_prosup")     return wbi::LocalId(iCub::skinDynLib::LEFT_ARM, 4, "l_wrist_prosup");
        if(jointName=="l_wrist_pitch")      return wbi::LocalId(iCub::skinDynLib::LEFT_ARM, 5, "l_wrist_pitch");
        if(jointName=="l_wrist_yaw")        return wbi::LocalId(iCub::skinDynLib::LEFT_ARM, 6, "l_wrist_yaw");
        if(jointName=="r_shoulder_pitch")   return wbi::LocalId(iCub::skinDynLib::RIGHT_ARM, 0, "r_shoulder_pitch");
        if(jointName=="r_shoulder_roll")    return wbi::LocalId(iCub::skinDynLib::RIGHT_ARM, 1, "r_shoulder_roll");
        if(jointName=="r_shoulder_yaw")     return wbi::LocalId(iCub::skinDynLib::RIGHT_ARM, 2, "r_shoulder_yaw");
        if(jointName=="r_elbow")            return wbi::LocalId(iCub::skinDynLib::RIGHT_ARM, 3, "r_elbow");
        if(jointName=="r_wrist_prosup")     return wbi::LocalId(iCub::skinDynLib::RIGHT_ARM, 4, "r_wrist_prosup");
        if(jointName=="r_wrist_pitch")      return wbi::LocalId(iCub::skinDynLib::RIGHT_ARM, 5, "r_wrist_pitch");
        if(jointName=="r_wrist_yaw")        return wbi::LocalId(iCub::skinDynLib::RIGHT_ARM, 6, "r_wrist_yaw");
        if(jointName=="l_hip_pitch")        return wbi::LocalId(iCub::skinDynLib::LEFT_LEG, 0, "l_hip_pitch");
        if(jointName=="l_hip_roll")         return wbi::LocalId(iCub::skinDynLib::LEFT_LEG, 1, "l_hip_roll");
        if(jointName=="l_hip_yaw")          return wbi::LocalId(iCub::skinDynLib::LEFT_LEG, 2, "l_hip_yaw");
        if(jointName=="l_knee")             return wbi::LocalId(iCub::skinDynLib::LEFT_LEG, 3, "l_knee");
        if(jointName=="l_ankle_pitch")      return wbi::LocalId(iCub::skinDynLib::LEFT_LEG, 4, "l_ankle_pitch");
        if(jointName=="l_ankle_roll")       return wbi::LocalId(iCub::skinDynLib::LEFT_LEG, 5, "l_ankle_roll");
        if(jointName=="r_hip_pitch")        return wbi::LocalId(iCub::skinDynLib::RIGHT_LEG, 0, "r_hip_pitch");
        if(jointName=="r_hip_roll")         return wbi::LocalId(iCub::skinDynLib::RIGHT_LEG, 1, "r_hip_roll");
        if(jointName=="r_hip_yaw")          return wbi::LocalId(iCub::skinDynLib::RIGHT_LEG, 2, "r_hip_yaw");
        if(jointName=="r_knee")             return wbi::LocalId(iCub::skinDynLib::RIGHT_LEG, 3, "r_knee");
        if(jointName=="r_ankle_pitch")      return wbi::LocalId(iCub::skinDynLib::RIGHT_LEG, 4, "r_ankle_pitch");
        if(jointName=="r_ankle_roll")       return wbi::LocalId(iCub::skinDynLib::RIGHT_LEG, 5, "r_ankle_roll");
        
        printf("ERROR Unknown global joint name: %s\n", jointName.c_str());
        return wbi::LocalId(iCub::skinDynLib::BODY_PART_UNKNOWN, 0);
    }
    
} // end namespace wbiIcub

#endif
