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

#ifndef IWHOLEBODYSTATES_H
#define IWHOLEBODYSTATES_H

#include <wbi/wbiConstants.h>

namespace wbi {
    
    class LocalId;
    class LocalIdList;
    
    /**
     * Interface to access the estimates of the state of the robot.
     */
    class iWholeBodyStates
    {
    public:
        /** Virtual destructor (to allow implementation of proper destructor in children classes). */
        virtual ~iWholeBodyStates();
        virtual bool init() = 0;
        virtual bool close() = 0;
        
        /** Add the specified estimate so that it can be read.
         * @param st Type of estimate.
         * @param sid Id of the estimate.
         * @return True if the estimate has been added, false otherwise (e.g. the estimate has been already added).
         */
        virtual bool addEstimate(const EstimateType st, const LocalId &sid) = 0;
        
        /** Add the specified estimates so that they can be read.
         * @param st Type of estimates.
         * @param sids Ids of the estimates.
         * @return True if the estimate has been added, false otherwise (e.g. the estimate has been already added).
         */
        virtual int addEstimates(const EstimateType st, const LocalIdList &sids) = 0;
        
        /** Remove the specified estimate.
         * @param st Type of the estimate to remove.
         * @param sid Id of the estimate to remove.
         * @return True if the estimate has been removed, false otherwise.
         */
        virtual bool removeEstimate(const EstimateType st, const LocalId &sid) = 0;
        
        /** Remove all the estimates associated to the specified joint.
         * @param j Id of the joint.
         * @return True if the operation succeeded, false otherwise.
         */
        //virtual bool removeEstimatesOfJoint(const LocalId &j);
        
        /** Get a copy of the estimate list of the specified estimate type.
         * @param st Type of estimate.
         * @return A copy of the estimate list. */
        virtual const LocalIdList& getEstimateList(const EstimateType st) = 0;
        
        /** Get the number of estimates of the specified type.
         * @return The number of estimates of the specified type. */
        //virtual int getEstimateNumber(const EstimateType st) = 0;
        
        /** Get the estimate of the specified quantity at the specified time.
         * @param et Type of estimate to get.
         * @param sid Id of the estimate
         * @param data Output data vector.
         * @param time Time at which to estimate the quantity.
         * @param blocking If true, perform a blocking read before estimating, otherwise the estimate is based on the last reading.
         * @return True if all the estimate succeeded, false otherwise.
         */
        virtual bool getEstimate(const EstimateType et, const LocalId &sid, double *data, double time=-1.0, bool blocking=true) = 0;
        
        /** Get all the estimates of the specified estimate type at the specified time.
         * @param et Type of estimate to get.
         * @param data Output data vector.
         * @param time Time at which to estimate the quantity.
         * @param blocking If true, perform a blocking read before estimating, otherwise the estimate is based on the last reading.
         * @return True if all the estimate succeeded, false otherwise.
         */
        virtual bool getEstimates(const EstimateType et, double *data, double time=-1.0, bool blocking=true) = 0;
        
        /** Set the value of the specified parameter of the estimation algorithm
         * of the specified estimate type.
         * @param et Estimation type (e.g. joint velocity, motor torque).
         * @param ep Parameter to set.
         * @param value Value of the parameter to set.
         * @return True if the operation succeeded, false otherwise. */
        virtual bool setEstimationParameter(const EstimateType et, const EstimationParameter ep, const void *value) = 0;
    };
    
}

#endif //IWHOLEBODYSTATES_H
