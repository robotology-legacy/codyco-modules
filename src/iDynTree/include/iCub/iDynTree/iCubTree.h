/*
 * Copyright (C) 2013 RobotCub Consortium
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU GPL v2.0 (or any later version)
 *
 */

#include <iCub/iDynTree/iDynTreeInterface.h>
#include <iCub/iDynTree/iDynTree.h>

#include <iostream>

#ifndef __ICUBTREE_H__
#define __ICUBTREE_H__

namespace iCub
{

namespace iDynTreeLib
{
    
struct iCubTree_version_tag
{
    int head_version;
    int legs_version;
    
    iCubTree_version_tag () 
    {
        head_version=1;
        legs_version=1;
    }
  
};

class iCubTree : public iDynTree 
{
	private:
		/**
		 * Get the partition of the iCub in a skinDynLib 
		 * 
		 */
		KDL::CoDyCo::TreePartition get_iCub_partition(const KDL::CoDyCo::TreeSerialization & icub_serialization);
	
	public:
	/**
	 * Default constructor for iCubTree
	 * 
	 * \note the FT sensor serialization is (0) LEFT_ARM (1) RIGHT_ARM (2) LEFT_LEG (3) RIGHT_LEG
	 * @param version a iCubTree_version_tag structure for defining the version of the parts
	 * 				  composing the iCubTree
	 */
	iCubTree(iCubTree_version_tag version, unsigned int verbose=0);
	
	~iCubTree();
};

}//end namespace

}

#endif
