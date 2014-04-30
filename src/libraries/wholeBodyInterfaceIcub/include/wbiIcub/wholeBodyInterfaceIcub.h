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

#ifndef WBI_ICUB_H
#define WBI_ICUB_H

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

#define INITIAL_TIMESTAMP -1000.0

#include "wbiIcub/icubWholeBodyActuators.h"
#include "wbiIcub/icubWholeBodyModel.h"
#include "wbiIcub/icubWholeBodySensors.h"
#include "wbiIcub/icubWholeBodyStates.h"
#include "wbiIcub/icubWholeBodyStatesLocal.h"
#include "wbiIcub/icubWholeBodyStatesRemote.h"

/* CODE UNDER DEVELOPMENT */

namespace wbiIcub
{

    const int JOINT_ESTIMATE_TYPES_SIZE = 3;
    ///< estimate types that are automatically added when calling addJoint(s) and automatically removed when calling removeJoint
    const wbi::EstimateType jointEstimateTypes[JOINT_ESTIMATE_TYPES_SIZE] =
    {
        wbi::ESTIMATE_JOINT_POS,         // joint position
        //wbi::ESTIMATE_JOINT_VEL,         // joint velocity
        //wbi::ESTIMATE_JOINT_ACC,         // joint acceleration
        wbi::ESTIMATE_JOINT_TORQUE,      // joint torque
        //wbi::ESTIMATE_MOTOR_VEL,         // motor velocity
        //wbi::ESTIMATE_MOTOR_TORQUE,      // motor torque
        wbi::ESTIMATE_MOTOR_PWM,         // motor PWM (proportional to motor voltage)
    };

    /**
     * Class to communicate with iCub.
     */
    class icubWholeBodyInterface : public wbi::wholeBodyInterface
    {
    protected:
        icubWholeBodyStates     *stateInt;
        icubWholeBodyActuators  *actuatorInt;
        icubWholeBodyModel      *modelInt;

    public:
        // *** CONSTRUCTORS ***
        icubWholeBodyInterface(const char* _name,
                               const char* _robotName,
                               iCub::iDynTree::iCubTree_version_tag icub_version /* = iCub::iDynTree::iCubTree_version_tag(2,2,true)*/);
        //icubWholeBodyInterface(const char* _name, const char* _robotName, std::string urdf_file_name);

        #ifdef CODYCO_USES_URDFDOM
        icubWholeBodyInterface(const char* _name,
                               const char* _robotName,
                               iCub::iDynTree::iCubTree_version_tag icub_version,
                               std::string urdf_file_name);
        #endif

        inline virtual ~icubWholeBodyInterface(){ close(); }
        virtual bool init();
        virtual bool close();
        virtual bool removeJoint(const wbi::LocalId &j);
        virtual bool addJoint(const wbi::LocalId &j);
        virtual int addJoints(const wbi::LocalIdList &j);

        // ACTUATORS
        virtual int getActuatorNumber(){                        return actuatorInt->getActuatorNumber(); }
        virtual bool removeActuator(const wbi::LocalId &j){     return actuatorInt->removeActuator(j); }
        virtual bool addActuator(const wbi::LocalId &j){        return actuatorInt->addActuator(j); }
        virtual int addActuators(const wbi::LocalIdList &j){    return actuatorInt->addActuators(j); }
        virtual const wbi::LocalIdList& getActuatorList(){      return actuatorInt->getActuatorList(); }
        virtual bool setControlMode(wbi::ControlMode cm, double *ref=0, int jnt=-1)
        { return actuatorInt->setControlMode(cm, ref, jnt); }
        virtual bool setControlReference(double *ref, int jnt=-1)
        { return actuatorInt->setControlReference(ref, jnt); }
        virtual bool setControlParam(wbi::ControlParam parId, const void *val, int jnt=-1)
        { return actuatorInt->setControlParam(parId, val, jnt); }
        virtual bool setActuactorConfigurationParameter(const std::string& parameterName, const yarp::os::Value& parameterValue);

        // STATES
        virtual bool addEstimate(const wbi::EstimateType st, const wbi::LocalId &sid){      return stateInt->addEstimate(st, sid); }
        virtual int addEstimates(const wbi::EstimateType st, const wbi::LocalIdList &sids){ return stateInt->addEstimates(st, sids); }
        virtual bool removeEstimate(const wbi::EstimateType st, const wbi::LocalId &sid){   return stateInt->removeEstimate(st, sid); }
        virtual const wbi::LocalIdList& getEstimateList(const wbi::EstimateType st){        return stateInt->getEstimateList(st); }
        virtual int getEstimateNumber(const wbi::EstimateType st){                          return stateInt->getEstimateNumber(st); }
        virtual bool getEstimate(const wbi::EstimateType et, const wbi::LocalId &sid, double *data, double time=-1.0, bool blocking=true)
        { return stateInt->getEstimate(et, sid, data, time, blocking); }
        virtual bool getEstimates(const wbi::EstimateType et, double *data, double time=-1.0, bool blocking=true)
        { return stateInt->getEstimates(et, data, time, blocking); }
        virtual bool setEstimationParameter(const wbi::EstimateType et, const wbi::EstimationParameter ep, const void *value)
        { return stateInt->setEstimationParameter(et, ep, value); }

        // MODEL
        virtual int getDoFs(){ return modelInt->getDoFs(); }
        virtual const wbi::LocalIdList& getJointList(){ return modelInt->getJointList(); }
        virtual bool getLinkId(const char *linkName, int &linkId)
        { return modelInt->getLinkId(linkName, linkId); }
        virtual bool getJointLimits(double *qMin, double *qMax, int joint=-1)
        { return modelInt->getJointLimits(qMin, qMax, joint); }
        virtual bool computeH(double *q, const wbi::Frame &xB, int linkId, wbi::Frame &H)
        { return modelInt->computeH(q, xB, linkId, H); }
        virtual bool computeJacobian(double *q, const wbi::Frame &xB, int linkId, double *J, double *pos=0)
        { return modelInt->computeJacobian(q, xB, linkId, J, pos); }
        virtual bool computeDJdq(double *q, const wbi::Frame &xB, double *dq, double *dxB, int linkId, double *dJdq, double *pos=0)
        { return modelInt->computeDJdq(q, xB, dq, dxB, linkId, dJdq, pos); }
        virtual bool forwardKinematics(double *q, const wbi::Frame &xB, int linkId, double *x)
        { return modelInt->forwardKinematics(q, xB, linkId, x); }
        virtual bool inverseDynamics(double *q, const wbi::Frame &xB, double *dq, double *dxB, double *ddq, double *ddxB, double *g, double *tau)
        { return modelInt->inverseDynamics(q, xB, dq, dxB, ddq, ddxB, g,tau); }
        virtual bool computeMassMatrix(double *q, const wbi::Frame &xB, double *M)
        { return modelInt->computeMassMatrix(q, xB, M); }
        virtual bool computeGeneralizedBiasForces(double *q, const wbi::Frame &xB, double *dq, double *dxB, double *g, double *h)
        { return modelInt->computeGeneralizedBiasForces(q, xB, dq, dxB, g, h); }
        virtual bool computeCentroidalMomentum(double *q, const wbi::Frame &xB, double *dq, double *dxB, double *h)
        { return modelInt->computeCentroidalMomentum(q, xB, dq, dxB, h); }
    };

} // end namespace wbiIcub

#endif

