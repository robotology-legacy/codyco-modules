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

}