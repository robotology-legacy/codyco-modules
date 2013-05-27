/*
 * Copyright (C) 2013 RobotCub Consortium
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version)
 *
 */

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
 */
class iDynTree: public iDynTreeInterface {
    private:
        KDL::Tree kinematic_tree;
        std::vector<KDL::Tree> dynamics_trees;
        
    public:
        /**
         * Constructor for iDynTree
         * 
         * @param _tree the KDL::Tree that must be used
         * @param joint_sensor_names the names of the joint that should 
         *        be considered as FT sensors
         *
         */
        iDynTree(KDL::Tree _tree, const std::vector<std::string> & joint_sensor_names);
    
}

}//end namespace

#endif



