/*
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Andrea Del Prete, Marco Randazzo
 * email: andrea.delprete@iit.it marco.randazzo@iit.it
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
 
 
#include "wbiIcub/wbiIcubUtil.h"

#include <algorithm>


namespace wbiIcub
{

bool loadBodyPartsFromConfig(yarp::os::Property & wbi_yarp_properties, std::vector<std::string> & body_parts_vector)
{
        yarp::os::Bottle parts_config = wbi_yarp_properties.findGroup("WBI_YARP_BODY_PARTS");
        const std::string numBodyPartsOption = "numBodyParts";
        if( !parts_config.check(numBodyPartsOption) ) {
            std::cout << "wbiIcub::loadBodyPartsFromConfig error: " << numBodyPartsOption << " option not found" << std::endl;
            return false;
        }
        int numBodyParts = parts_config.find(numBodyPartsOption).asInt();
        body_parts_vector.resize(numBodyParts);
        for(int bp=0; bp < numBodyParts; bp++ ) {
            std::ostringstream bodyPart_strm;
            bodyPart_strm<<"bodyPart"<<bp;
            std::string bodyPart = bodyPart_strm.str();
            if( ! parts_config.check(bodyPart) ) {
                std::cout << "wbiIcub::loadBodyPartsFromConfig error: " << bodyPart << " name not found" << std::endl;
                return false;
            }
            body_parts_vector[bp] = parts_config.find(bodyPart).asString().c_str();
        }   
        return true;
}

bool loadReverseTorsoJointsFromConfig(yarp::os::Property & wbi_yarp_properties, bool & reverse_torso_joints)
{
    if(  wbi_yarp_properties.findGroup("WBI_YARP_BODY_PARTS_REMAPPING").check("reverse_torso_joints") ) {
        reverse_torso_joints = true;
    } else {
        reverse_torso_joints = false;
    }
    return true;
}

bool loadSensorPortsFromConfig(yarp::os::Property & wbi_yarp_properties, 
                               const std::vector<std::string> & body_parts_vector,
                               std::vector<id_2_PortName> &ports,
                               const std::string group_name)
{
    yarp::os::Bottle ports_list = wbi_yarp_properties.findGroup(group_name);
    if( ports_list.isNull() || ports_list.size() == 0 ) {
        ports.resize(0);
        return true;
    }
    ports.resize(ports_list.size());
    for(int port_id = 0; port_id < ports_list.size(); port_id++ ) {
        yarp::os::Bottle * port = ports_list.get(port_id).asList();
        if( port == NULL || port->size() != 3 ) {
            std::cout << "wbiIcub::loadSensorPortsFromConfig error: " << ports_list.toString() << " has an element malformed element" << std::endl;
            return false;
        }
        std::string bodyPart = port->get(0).asString().c_str();
        int id = port->get(1).asInt();
        std::string port_name = port->get(2).asString().c_str();
        int body_part_index = std::find(body_parts_vector.begin(), body_parts_vector.end(), bodyPart) - body_parts_vector.begin();
        if( body_part_index >= (int)body_parts_vector.size() || body_part_index < 0 ) {
            std::cout << "wbiIcub::loadSensorPortsFromConfig error: bodyPart in " << port->toString() << " not recognized." << std::endl;
            return false;
        }
        id_2_PortName id_port_map;
        id_port_map.id = wbi::LocalId(body_part_index,id);
        id_port_map.portName = port_name;
        ports[port_id] = id_port_map;
    }
    return true;
}

bool loadFTSensorPortsFromConfig(yarp::os::Property & wbi_yarp_properties, 
                                 const std::vector<std::string> & body_parts_vector,
                                 std::vector<id_2_PortName> &ft_ports)
{
    return loadSensorPortsFromConfig(wbi_yarp_properties,body_parts_vector,ft_ports,"WBI_YARP_FT_PORTS");
}

bool loadIMUSensorPortsFromConfig(yarp::os::Property & wbi_yarp_properties, 
                                      const std::vector<std::string> & body_parts_vector, 
                                      std::vector<id_2_PortName> &imu_ports)
{
    return loadSensorPortsFromConfig(wbi_yarp_properties,body_parts_vector,imu_ports,"WBI_YARP_IMU_PORTS");
}

}