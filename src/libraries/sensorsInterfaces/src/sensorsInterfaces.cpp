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

#include <sensorsInterfaces/sensorsInterfaces.h>

using namespace std;
using namespace sensors;


//-----------------------------------------
iSensor::iSensor(): description("sensor")
{}
//-----------------------------------------
iSensor::iSensor(const std::string &_description): description(_description)
{}
//-----------------------------------------
iSensorInertial::iSensorInertial()
{
    init();
}
//-----------------------------------------
iSensorInertial::iSensorInertial(const std::string &_description): iSensor(_description)
{
    init();
}
//-----------------------------------------
void iSensorInertial::init()
{
    value.resize(6);
    value.clear();
}
//-----------------------------------------
iSensorFT6::iSensorFT6()
{
    init();
}
//-----------------------------------------
iSensorFT6::iSensorFT6(const std::string &_description): iSensor(_description)
{
    init();
}
//-----------------------------------------
void iSensorFT6::init()
{
    value.resize(6);
    value.clear();
}
//-----------------------------------------
//-----------------------------------------
//-----------------------------------------
//-----------------------------------------
//-----------------------------------------

