/*
 * Copyright (C) 2013 RobotCub Consortium
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version)
 *
 */

/**
 * \defgroup iDynTree iDynTree 
 *    
 * @ingroup icub_libraries
 *
 * \note <b>SI units adopted</b>: meters for lengths and radians
 *       for angles.
 *
 * \section dep_sec Dependencies 
 * - KDL
 * 
 * \section intro_sec Description
 * 
 * iDynTree is designed to be an efficient, generic and easy to use library
 * to calculate joint torques and external wrenches given kinetic information
 * (linear acceleration, angular velocity and  angular acceleration of a link, 
 * joint positions, velocities, acceleration) and embedded FT sensor measures, 
 * implementing the techniques described in this two papers:
 * 
 *     - [1] S. Ivaldi, M. Fumagalli, M. Randazzo, F. Nori, G. Metta, and G. Sandini
 *           Computing robot internal/external wrenches by means of inertial, tactile and f/t sensors: theory and implementation on the icub
 *           in Proc. of the 11th IEEE-RAS International Conference on Humanoid Robots, Bled, Slovenia, 2011.
 *           http://people.liralab.it/iron/Papers/conference/780_Ivaldi_etal2011.pdf
 *           http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=6100813
 * 
 *     - [2] A. Del Prete, L. Natale, F. Nori, and G. Metta,
 *           Contact Force Estimations Using Tactile Sensors and Force / Torque Sensors
 *           in Human Robot Interaction, 2012, pp. 0–2,
 *           http://pasa.liralab.it/pasapdf/1113_DelPrete_etal2012.pdf
 * 
 * Additional details (some not implemented, like multiple IMUs):  
 * 
 *     - [3] M. Fumagalli, S. Ivaldi, M. Randazzo, L. Natale, G. Metta, G. Sandini, and F. Nori, 
 *           Force feedback exploiting tactile and proximal force/torque sensing
 *           in Autonomous Robots, vol. 33, no. 4, pp. 381–398, 2012.
 *           http://dx.doi.org/10.1007/s10514-012-9291-2
 *           http://people.liralab.it/iron/Papers/journal/IvaldiFumagallietAl.pdf 
 * 
 * \section tested_os_sec Tested OS
 * 
 * Linux
 *
 * \section example_sec Example
 *
 * Exe
 *
 *
 * \author Silvio Traversaro
 * 
 * Copyright (C) 2013 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * 
 **/ 

#ifndef __IDYNTREEINTERFACE_H__
#define __IDYNTREEINTERFACE_H__

namespace iCub
{

namespace iDynTree
{
    
/**
 * \ingroup iDynTree
 *
 * An abstract class for calculating the torques and the external wrenches
 * in a Rigid Body Dynamic tree, using kinetic information (linear acceleration, angular velocity and
 * angular acceleration of one arbitrary link, joint positions, velocities, acceleration)
 * and FT sensor measures. 
 * 
 */
class iDynTreeInterface {
    
public:

    /** @name Set/Get methods for input quantities
     *  Methods to set and get input quantities
     */
    //@{

    /**
     * Set joint positions in the specified part (if no part 
     * is specified, set the joint positions of all the tree)
     * @param _q vector of joints position
     * @param part_name optional: the name of the part of joint to set
     * @return the effective joint positions, considering min/max values
     */
    virtual yarp::sig::Vector setAng(const yarp::sig::Vector & _q, const std::string & part_name="") = 0;
    
    /**
     * Get joint positions in the specified part (if no part 
     * is specified, get the joint positions of all the tree)
     * @param part_name optional: the name of the part of joints to set
     * @return vector of joint positions
     */
    virtual yarp::sig::Vector getAng(const std::string & part_name="") const =0;
    
    /**
     * Set joint speeds in the specified part (if no part 
     * is specified, set the joint speeds of all the tree)
     * @param _q vector of joint speeds
     * @param part_name optional: the name of the part of joints to set
     * @return the effective joint speeds, considering min/max values
     */
    virtual yarp::sig::Vector setDAng(const yarp::sig::Vector & _q, const std::string & part_name="")=0;
    
    /**
     * Get joint speeds in the specified part (if no part 
     * is specified, get the joint speeds of all the tree)
     * @param part_name optional: the name of the part of joints to get
     * @return vector of joint speeds
     */
    virtual yarp::sig::Vector getDAng(const std::string & part_name="") const = 0;
    
    /**
     * Set joint accelerations in the specified part (if no part 
     * is specified, set the joint accelerations of all the tree)
     * @param _q vector of joint speeds
     * @param part_name optional: the name of the part of joints to set
     * @return the effective joint accelerations, considering min/max values
     */
    virtual yarp::sig::Vector setD2Ang(const yarp::sig::Vector & _q, const std::string & part_name="")=0;
    
    /**
     * Get joint speeds in the specified part (if no part 
     * is specified, get the joint speeds of all the tree)
     * @param part_name optional: the name of the part of joints to get
     * @return vector of joint accelerations
     */
    virtual yarp::sig::Vector getD2Ang(const std::string & part_name="") const =0;
    
    /**
     * Set the inertial sensor measurements 
     * @param w0 a 3x1 vector with the initial/measured angular velocity
     * @param dw0 a 3x1 vector with the initial/measured angular acceleration
     * @param ddp0 a 3x1 vector with the initial/measured 3D proper (with gravity) linear acceleration
     * @return true if succeeds (correct vectors size), false otherwise
     */
    virtual bool setInertialMeasure(const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0)=0;
    
    /**
     * Get the inertial sensor measurements 
     * @param w0 a 3x1 vector with the initial/measured angular velocity
     * @param dw0 a 3x1 vector with the initial/measured angular acceleration
     * @param ddp0 a 3x1 vector with the initial/measured 3D proper (with gravity) linear acceleration
     * @return true if succeeds (correct vectors size), false otherwise
     */
    virtual bool getInertialMeasure(yarp::sig::Vector &w0, yarp::sig::Vector &dw0, yarp::sig::Vector &ddp0) const =0;
   
    /**
     * Set the FT sensor measurements on the specified sensor 
     * @param sensor_index the code of the specified sensor
     * @param ftm a 6x1 vector with forces and moments measured by the FT sensor
     * @return true if succeeds, false otherwise
     * 
     * \warning The convention used to serialize the wrench (Force-Torque) is different
     *          from the one used in Spatial Algebra (Torque-F)
     * 
     */
    virtual bool setSensorMeasurement(const int sensor_index, const yarp::sig::Vector &ftm)=0;
    
    /**
     * Get the FT sensor measurements on the specified sensor 
     * @param sensor_index the code of the specified sensor
     * @param ftm a 6x1 vector with forces (0:2) and moments (3:5) measured by the FT sensor
     * @return true if succeeds, false otherwise
     * 
     * \warning The convention used to serialize the wrench (Force-Torque) is different
     *          from the one used in Spatial Algebra (Torque-F)
     * 
     * \note if solveWrench(ignore_sensor_measures=true) is called, this
     *       function get the "simulated" measure of the sensor from the
     *       RNEA backward propagation of wrenches   
     */
    virtual bool getSensorMeasurement(const int sensor_index, yarp::sig::Vector &ftm) const =0;
    
    //@}
    
    /** @name Methods to execute phases of RNEA
     *  Methods to execute phases of Recursive Newton Euler Algorithm
     */
    //@{
    
    /**
     * Execute the kinematic phase (recursive calculation of position, velocity,
     * acceleration of each link) of the RNE algorithm.
     * @return true if succeeds, false otherwise
     */
    virtual bool kinematicRNEA()=0;    
    
    /**
     * Estimate the external contacts, supplied by the setContacts call
     * for each dynamical subtree
     * 
     */
    virtual bool estimateContactForces()=0;
    
    /**
     * Execute the dynamical phase (recursive calculation of internal wrenches
     * and of torques) of the RNEA algorithm for all the tree. 
     * @return true if succeeds, false otherwise
     */
    virtual bool dynamicRNEA()=0;
    
    //@}
    
    /** @name Get methods for output quantities
     *  Methods to get output quantities
     */
    //@{
    
    /**
     * Get joint torques in the specified part (if no part 
     * is specified, get the joint torques of all the tree)
     * @param part_name optional: the name of the part of joints to get
     * @return vector of joint torques
     */
    virtual yarp::sig::Vector getTorques(const std::string & part_name="") const=0;
    

    //@}
    /** @name Methods related to contact forces
     *  This methods are related both to input and output of the esimation:
     *  the iCub::skinDynLib::dynContactList is used both to specify the 
     *  unkown contacts via setContacts, and also to get the result of the
     *  estimation via getContacts
     * 
     *  \note If for a given subtree no contact is given, a default concact 
     *  is assumed, for example ad the end effector
     */
    //@{
    
    /**
     * Set the specified contacts 
     * @param contacts the list of the concacts on the iDynTree
     * @return true if operation succeeded, false otherwise
     */
    bool setContacts(const iCub::skinDynLib::dynContactList &contacts_list)=0;
    
    /**
     * Get the contacts list, containing the results of the estimation if
     * solveWrench() was called
     * @return A copy of the external contact list
     */
    const iCub::skinDynLib::dynContactList& getContacts() const=0;
    
    //@}
    
    //@}
    /** @name Methods related to Center of Mass calculation
     * 
     * 
     */
    //@{

    /**    
    * Performs the computation of the center of mass (COM) of the tree
    * @return true if succeeds, false otherwise
    */
    bool computeCOM()=0;
    
    /**
    * Performs the computation of the center of mass jacobian of the tree
    * @return true if succeeds, false otherwise
    */
    bool computeCOMjacobian()=0;
    
    /**
     * Get Center of Mass of the specified part (if no part 
     * is specified, get the joint torques of all the tree)
     * @param part_name optional: the name of the part of joints to get
     * @return Center of Mass vector
     */
    virtual yarp::sig::Vector getCOM(const std::string & part_name="") const=0;
    
    /**
     * Get Center of Mass Kacobian of the specified part (if no part 
     * is specified, get the joint torques of all the tree)
     * @param jac the output jacobiam matrix
     * @param part_name optional: the name of the part of joints to get
     * @return true if succeeds, false otherwise
     */
    virtual bool getCOMJacobian(const yarp::sig::Matrix & jac, const std::string & part_name="") const=0;

    
    //@}
}



    
}

}//end namespace

#endif



