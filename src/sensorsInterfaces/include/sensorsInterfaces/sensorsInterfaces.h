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
 * Authors: Serena Ivaldi
 * email: serena.ivaldi@isir.upmc.fr
 */

#ifndef SENSORSINTERFACES_H
#define SENSORSINTERFACES_H

#include <vector>
#include <map>
#include <string>
#include <vector>


/*
 * THIS CODE IS UNDER DEVELOPMENT!
 */


namespace sensors
{
    

    /*
     * Simple class to represent a generic sensor
     */
    class iSensor
    {
      public:
        std::string description;
        std::vector<double> value;
        //
        iSensor();
        iSensor(const std::string &_description);
        //
        virtual void init() = 0;
        virtual bool read(double *measure) = 0;
        virtual bool calibrate() = 0;
    };
    
    /*
     * Simple class to represent an inertial sensor
     */
    class iSensorInertial : public iSensor
    {
    public:
        //
        iSensorInertial();
        iSensorInertial(const std::string &_description);
        //
        virtual void init();
        virtual bool read(double *measure);
        virtual bool calibrate();
    };
    
    /*
     * Simple class to represent a 6 axis FT sensor
     */
    class iSensorFT6 : public iSensor
    {
    public:
        //
        iSensorFT6();
        iSensorFT6(const std::string &_description);
        //
        virtual void init();
        virtual bool read(double *measure);
        virtual bool calibrate();
    };
    
    
    
} // end namespace

#endif

