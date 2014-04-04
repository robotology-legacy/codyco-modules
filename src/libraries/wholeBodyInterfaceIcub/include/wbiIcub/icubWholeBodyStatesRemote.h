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
 
#ifndef WBSTATESREMOTE_ICUB_H
#define WBSTATESREMOTE_ICUB_H

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IVelocityControl2.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/filters.h>
#include <iCub/iDynTree/iCubTree.h>
#include <iCub/skinDynLib/skinContactList.h>
#include <wbiIcub/wbiIcubUtil.h>
#include <map>
 
 
namespace wbiIcub
{    
//     /**
//      * Class to access the estimates, by reading the estimation results from a remote module ( wholeBodyDynamics )
//      */
//    
//     class icubWholeBodyStatesRemote : public wbi::iWholeBodyStates
//     {
//     protected:
//         icubWholeBodySensors        *sensors;       // interface to access the robot sensors
//         icubWholeBodyDynamicsEstimator      *estimator;     // estimation thread
//         wbi::LocalIdList            emptyList;      ///< empty list of IDs to return in case of error
//         //double                      estWind;      // time window for the estimation
//         
//         virtual bool lockAndReadSensor(const wbi::SensorType st, const wbi::LocalId sid, double *data, double time, bool blocking);
//         virtual bool lockAndReadSensors(const wbi::SensorType st, double *data, double time, bool blocking);
//         virtual bool lockAndAddSensor(const wbi::SensorType st, const wbi::LocalId &sid);
//         virtual int lockAndAddSensors(const wbi::SensorType st, const wbi::LocalIdList &sids);
//         virtual bool lockAndRemoveSensor(const wbi::SensorType st, const wbi::LocalId &sid);
//         virtual wbi::LocalIdList lockAndGetSensorList(const wbi::SensorType st);
//         virtual int lockAndGetSensorNumber(const wbi::SensorType st);
// 
//         /** Get the velocity of the specified motor. */
//         bool getMotorVel(const wbi::LocalId &sid, double *data, double time, bool blocking);
//         /** Get the velocities of all the robot motors. */
//         bool getMotorVel(double *data, double time, bool blocking);
//         
//     public:
//         // *** CONSTRUCTORS ***
//         icubWholeBodyStatesRemote(const char* _name, const char* _robotName, double estimationTimeWindow);
//         inline virtual ~icubWholeBodyStatesRemote(){ close(); }
//         
//         virtual bool init();
//         virtual bool close();
//         
//         /** Add the specified estimate so that it can be read. 
//          * @param st Type of estimate.
//          * @param sid Id of the estimate.
//          * @return True if the estimate has been added, false otherwise (e.g. the estimate has been already added).
//          */
//         virtual bool addEstimate(const wbi::EstimateType st, const wbi::LocalId &sid);
//         
//         /** Add the specified estimates so that they can be read. 
//          * @param st Type of estimates.
//          * @param sids Ids of the estimates.
//          * @return True if the estimate has been added, false otherwise (e.g. the estimate has been already added).
//          */
//         virtual int addEstimates(const wbi::EstimateType st, const wbi::LocalIdList &sids);
// 
//         /** Remove the specified estimate. 
//          * @param st Type of the estimate to remove.
//          * @param j Id of the estimate to remove.
//          * @return True if the estimate has been removed, false otherwise.
//          */
//         virtual bool removeEstimate(const wbi::EstimateType st, const wbi::LocalId &sid);
//         
//         /** Get a copy of the estimate list of the specified estimate type.
//          * @param st Type of estimate.
//          * @return A copy of the estimate list. */
//         virtual const wbi::LocalIdList& getEstimateList(const wbi::EstimateType st);
//         
//         /** Get the number of estimates of the specified type.
//          * @return The number of estimates of the specified type. */
//         virtual int getEstimateNumber(const wbi::EstimateType st);
// 
//         /** Get the estimate of the specified quantity at the specified time.
//          * @param et Type of estimate to get.
//          * @param sid Id of the estimate
//          * @param data Output data vector.
//          * @param time Time at which to estimate the quantity.
//          * @param blocking If true, perform a blocking read before estimating, otherwise the estimate is based on the last reading.
//          * @return True if all the estimate succeeded, false otherwise.
//          */
//         virtual bool getEstimate(const wbi::EstimateType et, const wbi::LocalId &sid, double *data, double time=-1.0, bool blocking=true);
// 
//         /** Get all the estimates of the specified estimate type at the specified time.
//          * @param et Type of estimate to get.
//          * @param data Output data vector.
//          * @param time Time at which to estimate the quantity.
//          * @param blocking If true, perform a blocking read before estimating, otherwise the estimate is based on the last reading.
//          * @return True if all the estimate succeeded, false otherwise.
//          */
//         virtual bool getEstimates(const wbi::EstimateType et, double *data, double time=-1.0, bool blocking=true);
// 
//         /** Set the value of the specified parameter of the estimation algorithm
//          * of the specified estimate type.
//          * @param et Estimation type (e.g. joint velocity, motor torque).
//          * @param ep Parameter to set.
//          * @param value Value of the parameter to set.
//          * @return True if the operation succeeded, false otherwise. */
//         virtual bool setEstimationParameter(const wbi::EstimateType et, const wbi::EstimationParameter ep, const void *value);
//         
//         /** Get the estimated external force/torques 
//          * 
//          * \note temporary interface, should be substituted by properly defining an external force/torque estimate
//          * @param external_forces_list list of estimated external wrenches
//          * @return True if the operation succeeded, false otherwise.
//          */
//         virtual bool getEstimatedExternalForces(iCub::skinDynLib::skinContactList & external_forces_list);
// 
//     };
    
} 

#endif
