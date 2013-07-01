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
