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

#ifndef IWHOLEBODYSENSORS_H
#define IWHOLEBODYSENSORS_H

#include <wbi/wbiConstants.h>

namespace wbi {
    
    class LocalId;
    class LocalIdList;
    
    /*
     * Interface for reading the sensors of the robot.
     */
    class iWholeBodySensors
    {
    public:
        /** Virtual destructor (to allow implementation of proper destructor in children classes). */
        virtual ~iWholeBodySensors();
        
        /** Initialize the object. This method should be called after adding the sensors,
         *  but before reading any sensor. */
        virtual bool init() = 0;
        /** Close all the communication channels with the robot. This method should be
         *  called before destroying the object. */
        virtual bool close() = 0;
        
        /** Add the specified sensor so that it can be read.
         * @param st Type of sensor.
         * @param sid Id of the sensor.
         * @return True if the sensor has been added, false otherwise (e.g. the sensor has been already added).
         */
        virtual bool addSensor(const SensorType st, const LocalId &sid) = 0;
        
        /** Add the specified sensors so that they can be read.
         * @param st Type of sensors.
         * @param sids Ids of the sensors.
         * @return True if the sensor has been added, false otherwise (e.g. the sensor has been already added).
         */
        virtual int addSensors(const SensorType st, const LocalIdList &sids) = 0;
        
        /** Remove the specified sensor.
         * @param st Type of the sensor to remove.
         * @param sid Id of the sensor to remove.
         * @return True if the sensor has been removed, false otherwise.
         */
        virtual bool removeSensor(const SensorType st, const LocalId &sid) = 0;
        
        /** Remove all the sensors associated to the specified joint. This affects the reading of all the
         *  joint space sensors (e.g. encoders, pwm).
         * @param j Id of the joint.
         * @return True if the operation succeeded, false otherwise.
         */
        //virtual bool removeSensorsOfJoint(const LocalId &j);
        
        /** Get a copy of the sensor list of the specified sensor type.
         * @param st Type of sensors.
         * @return A copy of the sensor list. */
        virtual const LocalIdList& getSensorList(const SensorType st) = 0;
        
        /** Get the number of sensors of the specified type.
         * @return The number of sensors of the specified type. */
        //virtual int getSensorNumber(const SensorType st) = 0;
        
        /** Read the specified sensor.
         * @param st Type of sensor to read.
         * @param sid Id of the sensor to read.
         * @param data Output data vector.
         * @param stamps Output vector of timestamps.
         * @param blocking If true, the reading is blocking, otherwise it is not.
         * @return True if all the readings succeeded, false otherwise.
         */
        virtual bool readSensor(const SensorType st, const LocalId &sid, double *data, double *stamps=0, bool blocking=true) = 0;
        
        /** Read all the sensors of the specified type.
         * @param st Type of the sensor to read.
         * @param data Output data vector.
         * @param stamps Output vector of timestamps.
         * @param blocking If true, the reading is blocking, otherwise it is not.
         * @return True if the reading succeeded, false otherwise.
         */
        virtual bool readSensors(const SensorType st, double *data, double *stamps=0, bool blocking=true) = 0;
    };
}

#endif //IWHOLEBODYSENSORS_H
