/*
 * Copyright (C) 2013 RobotCub Consortium
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version)
 *
 */

#include <iCub/iDynTree/iDynTreeInterface.h>

#include <kdl_codyco/treeserialization.hpp>
#include <kdl_codyco/treepartition.hpp>
#include <kdl_codyco/treegraph.hpp>

#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>

#include <iostream>

#ifndef __IDYNTREE_H__
#define __IDYNTREE_H__

namespace iCub
{

namespace iDynTreeLib
{
    
    
    
/**
 * 
 * Data structure for containing information about internal FT sensors
 * To properly describe an FT sensor, it is necessary: 
 *      * the fixed junction of the FT sensor in the TreeGraph (a name)
 *      * the transformation from the *child* KDL::Tree link to the the reference
 *        frame of the FT measurement ( H_c_s ) such that given the measure f_s, the 
 *        wrench applied by the parent on the child expressed in the child frame ( f_c )
 *        is given by f_c = H_c_s f_s
 *         
 */
class FTSensor
{
    private:
		 const KDL::CoDyCo::TreeGraph * tree_graph;
         std::string fixed_joint_name;
         KDL::Frame H_child_sensor;
         int parent;
         int child;
         int sensor_id;
        
    public:
    
    		FTSensor(const KDL::CoDyCo::TreeGraph & _tree_graph, 
				 const std::string _fixed_joint_name,
				 const int _parent,
				 const int _child,
				 const int _sensor_id) : 
				 tree_graph(&_tree_graph),
				 fixed_joint_name(_fixed_joint_name),
				 H_child_sensor(KDL::Frame::Identity()),
				 parent(_parent),
				 child(_child),
				 sensor_id(_sensor_id) {}
    
		FTSensor(const KDL::CoDyCo::TreeGraph & _tree_graph, 
				 const std::string _fixed_joint_name,
				 const KDL::Frame _H_child_sensor,
				 const int _parent,
				 const int _child,
				 const int _sensor_id) : 
				 tree_graph(&_tree_graph),
				 fixed_joint_name(_fixed_joint_name),
				 H_child_sensor(_H_child_sensor),
				 parent(_parent),
				 child(_child),
				 sensor_id(_sensor_id) {}
		
		~FTSensor() {}							
		
        /**
         * For the given current_link, get the wrench excerted on the subgraph
         * as measured by the FT sensor
         */
        KDL::Wrench getWrenchExcertedOnSubGraph(int current_link, const std::vector<KDL::Wrench> & measured_wrenches )
        {
            if( current_link == parent ) {
				//The junction connected to an F/T sensor should be one with 0 DOF
				assert(tree_graph->getLink(child)->getAdjacentJoint(parent)->joint.getType() == KDL::Joint::None );
				KDL::Frame H_parent_child = tree_graph->getLink(child)->pose(parent,0.0);
                return H_parent_child*(H_child_sensor*measured_wrenches[sensor_id]);
            } else {
                assert(current_link == child);
                return H_child_sensor*measured_wrenches[sensor_id];
            }
        }
    
};

typedef std::vector<FTSensor> FTSensorList;
    
    
    
/**
 * \ingroup iDynTree
 *
 * An implementation of the iDynTreeInterface using KDL 
 * 
 * \todo add proper commented inherited methods
 * 
 * \warning The used velocities accelerations, without the information 
 *          about base linear velocity, are not the "real" ones. Are just
 *          the velocities assuming no base linear velocity, that by Galilean
 *          Relativity generate the same dynamics of the "real" ones
 * 
 */
class iDynTree : public iDynTreeInterface {
    private:
        KDL::CoDyCo::TreeGraph tree_graph; /**< TreeGraph object: it encodes the TreeSerialization and the TreePartition */
        KDL::CoDyCo::TreePartition partition; /**< TreePartition object explicit present as it is conventient to encode/decode dynContact objects */
        
        //Violating DRY principle, but for code clarity 
        int NrOfDOFs;
        int NrOfLinks;
        int NrOfFTSensors;
        int NrOfDynamicSubGraphs;
        
        //state of the robot
        KDL::JntArray q;
        KDL::JntArray dq;
        KDL::JntArray ddq;
        
        KDL::Twist imu_velocity;
        KDL::Twist imu_acceleration; /**< KDL acceleration: spatial proper acceleration */
        
        //dynContact stuff 
        std::vector< iCub::skinDynLib::dynContactList > contacts; /**< a vector of dynContactList, one for each dynamic subgraph */
        
        //Sensors measures
        std::vector< KDL::Wrench > measured_wrenches;
        FTSensorList ft_list;
        
        //Index representation of the Kinematic tree and the dynamics subtrees
        KDL::CoDyCo::Traversal kinematic_traversal;
        KDL::CoDyCo::Traversal dynamic_traversal;
        
        //Joint quantities
        KDL::JntArray torques;
        
        //Link quantities
		std::vector<KDL::Twist> v;
		std::vector<KDL::Twist> a;
        
        //External forces
        std::vector<KDL::Wrench> f_ext; /**< External wrench acting on a link */
        
		std::vector<KDL::Wrench> f; /**< Link wrench \warning it is traversal dependent */
		std::vector<KDL::Wrench> f_gi; /**< Gravitational and inertial wrench acting on a link */
        
        //iDynTreeContact data structures
        std::vector<int> link2subgraph_index; /**< for each link, return the correspondent dynamics subgraph index */
        std::vector<bool> link_is_subgraph_root; /**< for each link, return if it is a subgraph root */
        std::vector<int> subgraph_index2root_link; /**< for each subgraph, return the index of the root */
        std::vector< std::vector<FTSensor *> > link_FT_sensors; /**< for each link, return the list of FT_sensors connected to that link */
        bool are_contact_estimated;
        
        int getSubGraphIndex(int link_index) { return link2subgraph_index[link_index]; }
        bool isSubGraphRoot(int link_index) { return link_is_subgraph_root[link_index]; }
        
        int buildSubGraphStructure(const std::vector<std::string> & ft_names);
        
        /**
         * Get the A e b local to a plink, querying the contacts list and the FT sensor list
         * 
         */
        yarp::sig::Vector getLinkLocalAb_contacts(int global_index, yarp::sig::Matrix & A, yarp::sig::Vector & b); 
        
        bool isFTsensor(const std::string & joint_name, const std::vector<std::string> & ft_sensors) const;
        std::vector<yarp::sig::Matrix> A_contacts; /**< for each subgraph, the A regressor matrix of unknowns \todo use Eigen */
        std::vector<yarp::sig::Vector> b_contacts; /**< for each subgraph, the b vector of known terms \todo use Eigen */
        std::vector<yarp::sig::Vector> x_contacts; /**< for each subgraph, the x vector of unknowns */
        
        std::vector<KDL::Wrench> b_contacts_subtree; /**< for each link, the b vector of known terms of the subtree starting at that link expressed in the link frame*/
        
        /**
         * Preliminary version. If there are performance issues, this function
         * has several space for improvement.
         * 
         */
        void buildAb_contacts();
        
        /**
         * 
         * 
         */
        void store_contacts_results();
        
        /**
         * For a given link, returns the sum of the measure wrenches acting on
         * the link, expressed in the link reference frame
         * 
         */
        KDL::Wrench getMeasuredWrench(int link_id);

        //end iDynTreeContact data structures
        
        //Debug
        int verbose;
        
    public:
        /**
         * Constructor for iDynTree
         * 
         * @param _tree the KDL::Tree that must be used
         * @param joint_sensor_names the names of the joint that should 
         *        be considered as FT sensors
         * @param imu_link_name name of the link considered the IMU sensor
         * @param serialization (optional) an explicit serialization of tree links and DOFs
         * @param partition (optional) a partition of the tree (division of the links and DOFs in non-overlapping sets)
         *
         */
        iDynTree(const KDL::Tree & _tree, const std::vector<std::string> & joint_sensor_names, const std::string & imu_link_name, KDL::CoDyCo::TreeSerialization  serialization=KDL::CoDyCo::TreeSerialization(), KDL::CoDyCo::TreePartition partition=KDL::CoDyCo::TreePartition());
        
        ~iDynTree();
        
        
            /**
     * Set joint positions in the specified part (if no part 
     * is specified, set the joint positions of all the tree)
     * @param _q vector of joints position
     * @param part_name optional: the name of the part of joint to set
     * @return the effective joint positions, considering min/max values
     */
    virtual yarp::sig::Vector setAng(const yarp::sig::Vector & _q, const std::string & part_name="") ;
    
    /**
     * Get joint positions in the specified part (if no part 
     * is specified, get the joint positions of all the tree)
     * @param part_name optional: the name of the part of joints to set
     * @return vector of joint positions
     */
    virtual yarp::sig::Vector getAng(const std::string & part_name="") const;
    
    /**
     * Set joint speeds in the specified part (if no part 
     * is specified, set the joint speeds of all the tree)
     * @param _q vector of joint speeds
     * @param part_name optional: the name of the part of joints to set
     * @return the effective joint speeds, considering min/max values
     */
    virtual yarp::sig::Vector setDAng(const yarp::sig::Vector & _q, const std::string & part_name="");
    
    /**
     * Get joint speeds in the specified part (if no part 
     * is specified, get the joint speeds of all the tree)
     * @param part_name optional: the name of the part of joints to get
     * @return vector of joint speeds
     */
    virtual yarp::sig::Vector getDAng(const std::string & part_name="") const;
    
    /**
     * Set joint accelerations in the specified part (if no part 
     * is specified, set the joint accelerations of all the tree)
     * @param _q vector of joint speeds
     * @param part_name optional: the name of the part of joints to set
     * @return the effective joint accelerations, considering min/max values
     */
    virtual yarp::sig::Vector setD2Ang(const yarp::sig::Vector & _q, const std::string & part_name="");
    
    /**
     * Get joint speeds in the specified part (if no part 
     * is specified, get the joint speeds of all the tree)
     * @param part_name optional: the name of the part of joints to get
     * @return vector of joint accelerations
     */
    virtual yarp::sig::Vector getD2Ang(const std::string & part_name="") const;
    
    /**
     * Set the inertial sensor measurements 
     * @param w0 a 3x1 vector with the initial/measured angular velocity
     * @param dw0 a 3x1 vector with the initial/measured angular acceleration
     * @param ddp0 a 3x1 vector with the initial/measured 3D proper (with gravity) linear acceleration
     * @return true if succeeds (correct vectors size), false otherwise
     */
    virtual bool setInertialMeasure(const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0);
    
    /**
     * Get the inertial sensor measurements 
     * @param w0 a 3x1 vector with the initial/measured angular velocity
     * @param dw0 a 3x1 vector with the initial/measured angular acceleration
     * @param ddp0 a 3x1 vector with the initial/measured 3D proper (with gravity) linear acceleration
     * @return true if succeeds (correct vectors size), false otherwise
     */
    virtual bool getInertialMeasure(yarp::sig::Vector &w0, yarp::sig::Vector &dw0, yarp::sig::Vector &ddp0) const;
   
    /**
     * Set the FT sensor measurements on the specified sensor 
     * @param sensor_index the code of the specified sensor
     * @param ftm a 6x1 vector with forces (0:2) and moments (3:5) measured by the FT sensor
     * @return true if succeeds, false otherwise
     * 
     * \warning The convention used to serialize the wrench (Force-Torque) is different
     *          from the one used in Spatial Algebra (Torque-Force)
     * 
     */
    virtual bool setSensorMeasurement(const int sensor_index, const yarp::sig::Vector &ftm);
    
    /**
     * Get the FT sensor measurements on the specified sensor 
     * @param sensor_index the code of the specified sensor
     * @param ftm a 6x1 vector with forces (0:2) and moments (3:5) measured by the FT sensor
     * @return true if succeeds, false otherwise
     * 
     * \warning The convention used to serialize the wrench (Force-Torque) is different
     *          from the one used in Spatial Algebra (Torque-Force)
     * 
     * \note if dynamicRNEA() is called without before calling estimateContactForces() this
     *       function retrives the "simulated" measure of the sensor from the
     *       RNEA backward propagation of wrenches   
     */
    virtual bool getSensorMeasurement(const int sensor_index, yarp::sig::Vector &ftm) const;
    
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
    virtual bool kinematicRNEA();    
    
    /**
     * Estimate the external contacts, supplied by the setContacts call
     * for each dynamical subtree
     * 
     */
    virtual bool estimateContactForces();
    
    /**
     * Execute the dynamical phase (recursive calculation of internal wrenches
     * and of torques) of the RNEA algorithm for all the tree. 
     * @return true if succeeds, false otherwise
     */
    virtual bool dynamicRNEA();
    
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
    virtual yarp::sig::Vector getTorques(const std::string & part_name="") const;
    

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
     * Set the unknown contacts
     * @param contacts the list of the contacts on the iDynTree
     * @return true if operation succeeded, false otherwise
     */
    virtual bool setContacts(const iCub::skinDynLib::dynContactList &contacts_list);
    
    /**
     * Get the contacts list, containing the results of the estimation if
     * estimateContacts was called
     * @return A reference to the external contact list
     */
    virtual const iCub::skinDynLib::dynContactList getContacts() const;
    
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
    virtual bool computeCOM();
    
    /**
    * Performs the computation of the center of mass jacobian of the tree
    * @return true if succeeds, false otherwise
    */
    virtual bool computeCOMjacobian();
    
    /**
     * Get Center of Mass of the specified part (if no part 
     * is specified, get the joint torques of all the tree)
     * @param part_name optional: the name of the part of joints to get
     * @return Center of Mass vector
     */
    virtual yarp::sig::Vector getCOM(const std::string & part_name="") const;
    
    /**
     * Get Center of Mass Kacobian of the specified part (if no part 
     * is specified, get the joint torques of all the tree)
     * @param jac the output jacobiam matrix
     * @param part_name optional: the name of the part of joints to get
     * @return true if succeeds, false otherwise
     */
    virtual bool getCOMJacobian(const yarp::sig::Matrix & jac, const std::string & part_name="") const;

    
};

}//end namespace

}

#endif
