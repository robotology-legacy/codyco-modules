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
        KDL::Tree tree;
        
        std::
        
        std::vector<Frame> X;
        std::vector<Twist> S;
		std::vector<Twist> v;
		std::vector<Twist> a;
		std::vector<Wrench> f;
        
        JntArray torques;
        
    public:
        /**
         * Constructor for iDynTree
         * 
         * @param _tree the KDL::Tree that must be used
         * @param joint_sensor_names the names of the joint that should 
         *        be considered as FT sensors
         * @param imu_link_name name of the link considered the IMU sensor
         *
         */
        iDynTree(const KDL::Tree & _tree, const std::vector<std::string> & joint_sensor_names, const std::string & imu_link_name);
    
}

}//end namespace

#endif



