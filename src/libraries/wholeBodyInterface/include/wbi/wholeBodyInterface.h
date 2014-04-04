/*
 * Copyright (C) 2013  CoDyCo Consortium
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
 *
 * Authors: Andrea Del Prete, Silvio Traversaro, Marco Randazzo
 * email: andrea.delprete@iit.it - silvio.traversaro@iit.it - marco.randazzo@iit.it
 */

#ifndef WHOLEBODYINTERFACE_H
#define WHOLEBODYINTERFACE_H

#include <wbi/iWholeBodyStates.h>
#include <wbi/iWholeBodyModel.h>
#include <wbi/iWholeBodyActuators.h>

namespace wbi {
    /**
     * Interface to state estimations, kinematic/dynamic model and actuators of the robot.
     */
    class wholeBodyInterface: public iWholeBodyStates, public iWholeBodyModel, public iWholeBodyActuators
    {
    public:
        virtual ~wholeBodyInterface();
        virtual bool init() = 0;
        virtual bool close() = 0;
        
        /** Remove the actuator, model and all the estimates associated to the specified joint.
         * @param j Id of the joint.
         * @return True if the operation succeeded, false otherwise. */
        virtual bool removeJoint(const LocalId &j) = 0;
        
        /** Add the actuator, model and all the estimates associated to the specified joint.
         * @param j Id of the joint.
         * @return True if the operation succeeded, false otherwise. */
        virtual bool addJoint(const LocalId &j) = 0;
        
        /** Add the actuators, models and all the estimates associated to the specified joints.
         * @param j Id of the joint.
         * @return True if the operation succeeded, false otherwise. */
        virtual int addJoints(const LocalIdList &j) = 0;
    };
}

#endif //WHOLEBODYINTERFACE_H
