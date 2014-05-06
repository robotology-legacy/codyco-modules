/*
 * Copyright (C) 2013 RBCS Department & iCub Facility - Istituto Italiano di Tecnologia
 * Author: Andrea Del Prete, Francesco Romano, Silvio Traversaro
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

#ifndef WBMODEL_ICUB_H
#define WBMODEL_ICUB_H

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
  /**
     * Interface to the kinematic/dynamic model of iCub.
     */
    class icubWholeBodyModel: public wbi::iWholeBodyModel
    {
    protected:
        wbi::LocalIdList jointIdList;
        int dof;

        iCub::iDynTree::DynTree * p_model;

        iCub::iDynTree::iCubTree * p_icub_model;


        iCub::iDynTree::iCubTree_version_tag version;

        yarp::sig::Matrix world_base_transformation;

        yarp::sig::Vector v_base, a_base;
        yarp::sig::Vector omega_base, domega_base;

        yarp::sig::Vector v_six_elems_base;
        yarp::sig::Vector a_six_elems_base;


        yarp::sig::Vector all_q;
        yarp::sig::Vector all_dq;
        yarp::sig::Vector all_ddq;

        yarp::sig::Vector all_q_min, all_q_max;

        //Output buffers
        yarp::sig::Vector generalized_floating_base_torques; //n+6 outputs for inverse dynamics
        yarp::sig::Matrix floating_base_mass_matrix;
        yarp::sig::Matrix reduced_floating_base_mass_matrix;
        yarp::sig::Vector six_elem_buffer;
        yarp::sig::Vector three_elem_buffer;

        // *** Variables needed for opening IControlLimits interfaces
        std::string                                 name;           // name used as root for the local ports
        std::string                                 robot;          // name of the robot
        std::vector<int>                            bodyParts;      // list of the body parts
        std::vector<std::string>                    bodyPartNames;  // names of the body parts
        std::map<int, yarp::dev::PolyDriver*>       dd;
        std::map<int, yarp::dev::IControlLimits*>   ilim;

        bool reverse_torso_joints;

        bool initDriversDone;
        
        bool openDrivers(int bp);

        int bodyPartJointMapping(int bodypart_id, int local_id);

        bool convertBasePose(const wbi::Frame &xBase, yarp::sig::Matrix & H_world_base);
        bool convertBaseVelocity(const double *dxB, yarp::sig::Vector & v_b, yarp::sig::Vector & omega_b);
        bool convertBaseAcceleration(const double *ddxB, yarp::sig::Vector & a_b, yarp::sig::Vector & domega_b);
        bool convertBaseVelocity(const double *dxB, yarp::sig::Vector & v_six_elems_b);
        bool convertBaseAcceleration(const double *ddxB, yarp::sig::Vector & a_six_elems_b);


        bool convertQ(const double *q_input, yarp::sig::Vector & q_complete_output);
        bool convertQ(const yarp::sig::Vector & q_complete_input, double *q_output);
        bool convertDQ(const double *dq_input, yarp::sig::Vector & dq_complete_output);
        bool convertDDQ(const double *ddq_input, yarp::sig::Vector & ddq_complete_output);

        bool convertGeneralizedTorques(yarp::sig::Vector idyntree_base_force, yarp::sig::Vector idyntree_torques, double * tau);

    public:
         // *** CONSTRUCTORS ***
        /**
          * @param _name Local name of the interface (used as stem of port names)
          * @param _robotName Name of the robot
          * @param icub_version version of the iCub (default: head 2 legs 2 feet_ft true)
          * @param initial_q the initial value for all the 32 joint angles (default: all 0)
          * @param _bodyPartNames Vector of names of the body part (used when opening the polydrivers)
          */
        icubWholeBodyModel(const char* _name,
                           const char* _robotName,
                           const iCub::iDynTree::iCubTree_version_tag icub_version,
                           double* initial_q=0,
                           const std::vector<std::string> &_bodyPartNames=std::vector<std::string>(iCub::skinDynLib::BodyPart_s,iCub::skinDynLib::BodyPart_s+sizeof(iCub::skinDynLib::BodyPart_s)/sizeof(std::string)));


        #ifdef CODYCO_USES_URDFDOM
         /**
          * @param _name Local name of the interface (used as stem of port names)
          * @param _robotName Name of the robot
          * @param icub_version version of the iCub (default: head 2 legs 2 feet_ft true)
          * @param urdf_file urdf file representing the icub model
          * @param initial_q the initial value for all the 32 joint angles (default: all 0)
          * @param _bodyPartNames Vector of names of the body part (used when opening the polydrivers)
          */
        icubWholeBodyModel(const char* _name,
                           const char* _robotName,
                           const iCub::iDynTree::iCubTree_version_tag icub_version,
                           const std::string urdf_file,
                           double* initial_q=0,
                           const std::vector<std::string> &_bodyPartNames=std::vector<std::string>(iCub::skinDynLib::BodyPart_s,iCub::skinDynLib::BodyPart_s+sizeof(iCub::skinDynLib::BodyPart_s)/sizeof(std::string)));



         /**
          * @param _name Local name of the interface (used as stem of port names)
          * @param _robotName Name of the robot
          * @param urdf_file path to the urdf file describing the dynamics model
          * @param initial_q the initial value for all the 32 joint angles (default: all 0)
          * @param wbi_yarp_conf the yarp::os::Property containg the options for wbi
          * @param _bodyPartNames Vector of names of the body part (used when opening the polydrivers)
          */
        icubWholeBodyModel(const char* _name,
                           const char* _robotName,
                           const char* _urdf_file,
                           yarp::os::Property & wbi_yarp_conf,
                           double* initial_q=0);
        #endif

        inline virtual ~icubWholeBodyModel(){ close(); }
        virtual bool init();
        virtual bool close();

        inline virtual int getDoFs(){ return dof; }

        /** Remove the specified joint form the robot model. The joint is considered blocked
          * at its last known value (values of the joint angles are stored whenever a method of
          * iWholeBodyModel is called). If no previous value of the joint angle is known, zero is assumed.
          * @param j Id of the joint to remove
          * @return True if the joint was found and removed, false otherwise. */
        virtual bool removeJoint(const wbi::LocalId &j);
        virtual bool addJoint(const wbi::LocalId &j);
        virtual int addJoints(const wbi::LocalIdList &j);
        virtual const wbi::LocalIdList& getJointList(){    return jointIdList; }

        virtual bool getJointLimits(double *qMin, double *qMax, int joint=-1);

        /** Get the id of the link with the specified name.
          * @param linkName Name of the link.
          * @param linkId Id of the link (if found).
          * @return True if the link name was found, false otherwise. */
        inline virtual bool getLinkId(const char *linkName, int &linkId)
        { linkId = p_icub_model->getLinkIndex(linkName); return linkId>=0; }

        /** Compute rototranslation matrix from root reference frame to reference frame associated to the specified link.
          * @param q Joint angles
          * @param xBase Rototranslation from world frame to robot base frame
          * @param linkId Id of the link that is the target of the rototranslation
          * @param H Output 4x4 rototranslation matrix (stored by rows)
          * @return True if the operation succeeded, false otherwise (invalid input parameters) */
        virtual bool computeH(double *q, const wbi::Frame &xBase, int linkId, wbi::Frame &H);

        /** Compute the Jacobian of the specified point of the robot.
          * @param q Joint angles
          * @param xBase Rototranslation from world frame to robot base frame
          * @param linkId Id of the link
          * @param J Output 6xN Jacobian matrix (stored by rows), where N=number of joints
          * @param pos 3d position of the point expressed w.r.t the link reference frame
          * @return True if the operation succeeded, false otherwise (invalid input parameters)
          * @note If linkId==COM_LINK_ID then the angular part of J is related to the angular velocity of the
          *       whole multi-body system. This Jacobian premultiplied by the whole robot's 6D inertia
          *       matrix is equal to the Jacobian of the angular momentum of the whole robot. */
        virtual bool computeJacobian(double *q, const wbi::Frame &xBase, int linkId, double *J, double *pos=0);

        /** Given a point on the robot, compute the product between the time derivative of its
          * Jacobian and the joint velocity vector.
          * @param q Joint angles
          * @param xBase Rototranslation from world frame to robot base frame
          * @param dq Joint velocities
          * @param linkID ID of the link
          * @param dJdq Output 6-dim vector containing the product dJ*dq
          * @param pos 3d position of the point expressed w.r.t the link reference frame
          * @return True if the operation succeeded, false otherwise (invalid input parameters)
          * \note If linkId==COM_LINK_ID only the first three elements of dJdq (the linear part) are computed,
          *          the angular part is zero
          */
        virtual bool computeDJdq(double *q, const wbi::Frame &xBase, double *dq, double *dxB, int linkID, double *dJdq, double *pos=0);

        /** Compute the forward kinematics of the specified joint.
          * @param q Joint angles.
          * @param xBase Rototranslation from world frame to robot base frame
          * @param linkId Id of the link.
          * @param x Output 7-dim pose vector (3 for pos, 4 for quaternion orientation).
          * @return True if the operation succeeded, false otherwise. */
        virtual bool forwardKinematics(double *q, const wbi::Frame &xBase, int linkId, double *x);

        /** Compute the inverse dynamics.
          * @param q Joint angles.
          * @param xBase Rototranslation from world frame to robot base frame
          * @param dq Joint velocities.
          * @param dxB Velocity of the robot base, 3 values for linear velocity and 3 values for angular velocity.
          * @param ddq Joint accelerations.
          * @param ddxB Acceleration of the robot base, 3 values for linear acceleration and 3 values for angular acceleration.
          * @param g gravity acceleration expressed in world frame (3 values)
          * @param tau Output joint torques.
         * @return True if the operation succeeded, false otherwise. */
        virtual bool inverseDynamics(double *q, const wbi::Frame &xBase, double *dq, double *dxB, double *ddq, double *ddxB, double *g, double *tau);

        /** Compute the floating base Mass Matrix.
         * @param q Joint angles (rad).
         * @param xBase Rototranslation from world frame to robot base frame
         * @param M Output N+6xN+6 mass matrix, with N=number of joints.
         * @return True if the operation succeeded, false otherwise.
         */
        virtual bool computeMassMatrix(double *q, const wbi::Frame &xBase, double *M);

        /** Compute the generalized bias forces (gravity+Coriolis+centrifugal) terms.
         * @param q Joint angles (rad).
         * @param xBase Rototranslation from world frame to robot base frame
         * @param dq Joint velocities (rad/s).
         * @param dxB Velocity of the robot base in world reference frame, 3 values for linear and 3 for angular velocity.
         * @param g gravity acceleration expressed in world frame (3 values)
         * @param h Output N+6-dim vector containing all generalized bias forces (gravity+Coriolis+centrifugal), with N=number of joints.
         * @return True if the operation succeeded, false otherwise. */
        virtual bool computeGeneralizedBiasForces(double *q, const wbi::Frame &xBase, double *dq, double *dxB, double*g, double *h);

        /** Compute the 6 element centroidal momentum, as defined in:
         * Centroidal dynamics of a humanoid robot - DE Orin, A Goswami, SH Lee - Autonomous Robots 35 (2-3), 161-176
         * @param q Joint angles (in radians)
         * @param xBase Rototranslation from world frame to robot base frame (\f${}^w H_b \f$)
         * @param dq Joint velocities (rad/s).
         * @param dxB Velocity of the robot base in world reference frame, 3 values for linear and 3 for angular velocity
         * @param h output 6-element vector containg the centroidal momentum (3 value for linear momentum and 3 for angular momentum)
         * @return True if the operation succeeded, false otherwise. */
        virtual bool computeCentroidalMomentum(double *q, const wbi::Frame &xBase, double *dq, double *dxB, double *h);


    };
}

#endif
