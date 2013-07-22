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
 * Authors: Andrea Del Prete, Marco Randazzo
 * email: andrea.delprete@iit.it - marco.randazzo@iit.it
 */

#include <wbi/wbi.h>

using namespace std;
using namespace wbi;

LocalIdList::LocalIdList(){}

LocalIdList::LocalIdList(int bp, vector<int>){}

/** Convert a local id into a global id */
int LocalIdList::localToGlobalId(const LocalId &i)
{
    return 0;
}

/** Convert a global id into a local id */
LocalId LocalIdList::globalToLocalId(int globalId)
{
    return LocalId();
}

//Remove an existing joint 
bool LocalIdList::removeId(const LocalId &j)
{
    LocalIdList::iterator itBp = find(j.bodyPart);
    if(itBp == end())
        return false;
    FOR_ALL_JOINTS_NC(itBp, itJ)
    {
        if(j.index == *itJ)
        {
            itBp->second.erase(itJ);
            return true;
        }
    }
    return false;
}

bool LocalIdList::addId(const LocalId &i)
{
    if(containsId(i))
        return false;
    (*this)[i.bodyPart].push_back(i.index);
    return true;
}

int LocalIdList::addIdList(const LocalIdList &jList)
{
    int count = 0;
    FOR_ALL_OF(itBp, itJ, jList)
        if(!containsId(LocalId(itBp->first,*itJ)))
        {
            (*this)[itBp->first].push_back(*itJ);
            count++;
        }
    return count;
}

bool LocalIdList::containsId(const LocalId &i)
{
    if(find(i.bodyPart)==end())
        return false;
    vector<int> &v = (*this)[i.bodyPart];
    for(unsigned int j=0; j<v.size(); j++)
        if(v[j]==i.index)
            return true;
    return false;
}

// Get the number of ids in this list
unsigned int LocalIdList::size()
{
    unsigned int s=0;
    FOR_ALL_BODY_PARTS_OF(itBp, (*this))
        s += itBp->second.size();
    return s;
}