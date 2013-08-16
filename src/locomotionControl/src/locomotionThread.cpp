/* 
 * Copyright (C) 2013 CoDyCo
 * Author: Andrea Del Prete
 * email:  andrea.delprete@iit.it
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

#include <locomotion\locomotionThread.h>

using namespace locomotion;

LocomotionThread::LocomotionThread(string _name, string _robot, int _period)
    :  RateThread(_period), name(_name), robot(_robot)
{

}

bool LocomotionThread::threadInit()
{
    return true;
}

void LocomotionThread::run()
{

}

void LocomotionThread::threadRelease()
{

}
