/* 
 * Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia - Italian Institute of Technology
 * Author: Silvio Traversaro
 * email:  silvio.traversaro@iit.it
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

#include "wholeBodyDynamicsTree/wholeBodyDynamicsThread.h"
#include <wbiIcub/wholeBodyInterfaceIcub.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/math/SVD.h>


using namespace yarp::math;
using namespace wbiIcub;

//************************************************************************************************************************
wholeBodyDynamicsThread::wholeBodyDynamicsThread(string _name, string _robotName, int _period, icubWholeBodyStatesLocal *_wbi)
    :  RateThread(_period), name(_name), robotName(_robotName), robot(_wbi)
{
}

//*************************************************************************************************************************
bool wholeBodyDynamicsThread::threadInit()
{
    printf("\n\n");
    return true;
}

//*************************************************************************************************************************
void wholeBodyDynamicsThread::run()
{
    //Get data for estimator
   
    //if normal mode, publish the 
    printCountdown = (printCountdown>=PRINT_PERIOD) ? 0 : printCountdown +(int)getRate();   // countdown for next print (see sendMsg method)
}

//*****************************************************************************
void wholeBodyDynamicsThread::threadRelease()
{

}

//*****************************************************************************
void wholeBodyDynamicsThread::closePort(Contactable *_port)
{
    if (_port)
    {
        _port->interrupt();
        _port->close();

        delete _port;
        _port = 0;
    }
}

//*****************************************************************************
template <class T> void wholeBodyDynamicsThread::broadcastData(T& _values, BufferedPort<T> *_port)
{
    if (_port && _port->getOutputCount()>0)
    {
        _port->setEnvelope(this->timestamp);
        _port->prepare()  = _values ;
        _port->write();
    }
}

//*****************************************************************************
void wholeBodyDynamicsThread::writeTorque(Vector _values, int _address, BufferedPort<Bottle> *_port)
{
    Bottle a;
    a.addInt(_address);
    for(size_t i=0;i<_values.length();i++)
        a.addDouble(_values(i));
    _port->prepare() = a;
    _port->write();
}