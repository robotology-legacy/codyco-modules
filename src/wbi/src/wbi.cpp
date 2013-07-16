/*
 * Copyright (C) 2013  CoDyCo Consortium
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
 *
 * Authors: Serena Ivaldi, Andrea Del Prete, Marco Randazzo
 * email: serena.ivaldi@isir.upmc.fr - andrea.delprete@iit.it - marco.randazzo@iit.it
 */

#include <wbi/wbi.h>

using namespace std;
using namespace wbi;


//Remove an existing joint 
bool JointIds::removeJoint(const JointId &j)
{
    JointIds::iterator itBp = find(j.bp);
    if(itBp == end())
        return false;
    FOR_ALL_JOINTS_NC(itBp, itJ)
    {
        if(j.joint == *itJ)
        {
            itBp->second.erase(itJ);
            return true;
        }
    }
    return false;
}

//Return true if the added joint belongs to a body part that wasn't present before
bool JointIds::addJoint(const JointId &j)
{
    bool newBodyPart = !(find(j.bp)==end());
    operator[](j.bp).push_back(j.joint);
    return newBodyPart;
}

//Return true if at least one of the added joints belong to a body part that wasn't present before
bool JointIds::addJoints(const JointIds &jList)
{
    bool ok = true;
    for(JointIds::const_iterator it=jList.begin(); it!=jList.end(); it++)
    {
        if(find(it->first)==end())
            ok = false;
        FOR_ALL_JOINTS(it, itJ)
            operator[](it->first).push_back(*itJ);
    }
    return ok;
}

// Get the number of degrees of freedom
unsigned int JointIds::getDoFs()
{
    unsigned int dof=0;
    FOR_ALL_BODY_PARTS_OF(itBp, (*this))
        dof += itBp->second.size();
    return dof;
}