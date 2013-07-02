/*
 * Copyright (C) 2013 RobotCub Consortium
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version)
 *
 */

#include <kdl_codyco/treeserialization.hpp>
#include <kdl_codyco/treepartition.hpp>

#ifndef __IDYNTREE_H__
#define __IDYNTREE_H__

namespace iCub
{

namespace iDynTree
{
    
    
    
/**
 * 
 * Data structure for containing information about internal FT sensors
 * To properly describe an FT sensor, it is necessary: 
 *      * the fixed joint of the FT sensor in the TreeGraph (a name)
 *      * the transformation from the *parent* KDL::Tree link to the the reference
 *        frame of the FT measurement ( H_p_s ) such that given the measure f_s, the 
 *        wrench applied by the parent to the child expressed in the parent frame ( f_p )
 *        is given by f_p = H_p_s f_s
 *         
 */
class FTSensor
{
    private:
        const std::string fixed_joint_name;
        KDL::Frame H_parent_sensor;
        int parent;
        int child;
        
    public:
        /**
         * For the given current_link, get the wrench excerted on the subgraph
         * as measured by the FT sensor
         */
        KDL::Frame getWrenchExcertedOnSubGraph(int current_link)
        {
            if( current_link == parent ) {
                return -H_parent_sensor;
            } else {
                assert(current_link == child);
                //return H_parent_sensor.Inverse();
            }
        }
    
}

typedef std::vector<FTSensor> FTSensorList;
    
    
    
/**
 * \ingroup iDynTree
 *
 * An implementation of the iDynTreeInterface using KDL 
 * 
 * \warning The used velocities accelerations, without the information 
 *          about base linear velocity, are not the "real" ones. Are just
 *          the velocities assuming no base linear velocity, that by Galilean
 *          Relativity generate the same dynamics of the "real" ones
 * 
 */
class iDynTree: public iDynTreeInterface {
    private:
        KDL::CoDyCo::TreeGraph tree_graph; /**< TreeGraph object: it encodes the TreeSerialization and the TreePartition */
        KDL::CoDyCo::TreePartition partition; /**< TreePartition object explicit present as it is conventient to encode/decode dynContact objects */
        
        //Violating DRY principle, but for code clarity 
        int NrOfDOFs;
        int NrOfLinks;
        int NrOfFTSensors;
        int NrOfDynamicSubTrees;
        
        //state of the robot
        KDL::JntArray q;
        KDL::JntArray dq;
        KDL::JntArray ddq;
        
        KDL::Twist imu_velocity;
        KDL::Twist imu_acceleration; /**< KDL acceleration: spatial proper acceleration */
        
        //\todo add dynContact stuff 
        iCub::skinDynLib::dynContactList contacts;
        
        //Sensors measures
        std::vector< KDL::Wrench > measured_wrenches;
        
        //Index representation of the Kinematic tree and the dynamics subtrees
        Traversal kinematic_traversal;
        Traversal dynamic_traversal;
        
        //Joint quantities
        JntArray torques;
        
        //Link quantities
		std::vector<Twist> v;
		std::vector<Twist> a;
        
		std::vector<Wrench> f;
        
        //iDynTreeContact data structures
        std::vector<int> link2subgraph_index; /**< for each link, return the correspondent dynamics subgraph index */
        
        int getSubGraphIndex(int link_index) { return link2subgraph_index[link_index]; }
        void buildSubGraphStructure();
        bool isFTsensor(const std::string & joint_name, const std::vector<std::string> & ft_sensors) const;
        
        std::vector<yarp::sig::Matrix> A_contacts; /**< for each subgraph, the A regressor matrix of unknowns \todo use Eigen */
        std::vector<yarp::sig::Vector> b_contacts; /**< for each subgraph, the b vector of known terms \todo use Eigen */
        //end iDynTreeContact data structures
        
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
        iDynTree(const KDL::Tree & _tree, const std::vector<std::string> & joint_sensor_names, const std::string & imu_link_name, TreeSerialization  serialization=TreeSerialization(), TreePartion partition=TreePartition());
        
        ~iDynTree();
    
}

}//end namespace

#endif
