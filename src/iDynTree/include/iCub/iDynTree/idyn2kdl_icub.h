/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */

#ifndef IDYN2KDL_ICUB_H 
#define IDYN2KDL_ICUB_H 

#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <kdl/tree.hpp>
#include <iCub/iDynTree/iDyn2KDL.h>
#include <iCub/iDynTree/iCubTree.h>

/**
 * Get a KDL::Tree model from a iCub::iDyn::iCubWholeBody object
 * 
 * @param icub_idyn the iCub::iDyn::iCubWholeBody iCub object
 * @param icub_kdl the iCub KDL::Tree object
 * @return false in case of error, true otherwise
 */
bool toKDL(const iCub::iDyn::iCubWholeBody & icub_idyn, KDL::Tree & icub_kdl, iCub::iDynTree::iCubTree_serialization_tag serial, bool debug=false);

bool toKDL_iDynDebug(const iCub::iDyn::iCubWholeBody & icub_idyn, KDL::Tree & icub_kdl, bool debug=false);


#endif
