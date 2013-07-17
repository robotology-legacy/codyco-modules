/*
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 */

#include <iCub/iDynTree/DynTree.h>

#include <iCub/iDynTree/yarp_kdl.h>

#include <kdl_codyco/position_loops.hpp>
#include <kdl_codyco/rnea_loops.hpp>

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <vector>

using namespace yarp::sig;
using namespace yarp::math;

/**
 * 
 * \todo implement range checking and part handling
 * \todo iDynTreeContact solve dynamic allocation of regressor matrix for contacts
 * 
 */ 
 
namespace iCub {
namespace iDynTree {
	
DynTree::DynTree()
{
}

DynTree::DynTree(const KDL::Tree & _tree, 
                   const std::vector<std::string> & joint_sensor_names,
                   const std::string & imu_link_name,
                   KDL::CoDyCo::TreeSerialization serialization,
                   KDL::CoDyCo::TreePartition _partition
                   ):  tree_graph(_tree,serialization,_partition)
{
	constructor(_tree,joint_sensor_names,imu_link_name,serialization,_partition);
}

void DynTree::constructor(const KDL::Tree & _tree, 
					  const std::vector<std::string> & joint_sensor_names,
                      const std::string & imu_link_name,
                      KDL::CoDyCo::TreeSerialization serialization,
                      KDL::CoDyCo::TreePartition _partition,
                      std::vector<KDL::Frame> child_sensor_transforms
					  )
{
	int ret;
    
    tree_graph = KDL::CoDyCo::TreeGraph(_tree,serialization,_partition);  
    
    //Setting useful constants
    NrOfDOFs = _tree.getNrOfJoints();
    NrOfLinks = _tree.getNrOfSegments();
    NrOfFTSensors = joint_sensor_names.size();
    NrOfDynamicSubGraphs = NrOfFTSensors + 1;
    
    assert((int)tree_graph.getNrOfDOFs() == NrOfDOFs);
    assert((int)tree_graph.getNrOfLinks() == NrOfLinks);
    
    q = KDL::JntArray(NrOfDOFs);
    dq = KDL::JntArray(NrOfDOFs);
    ddq = KDL::JntArray(NrOfDOFs);
    
    torques = KDL::JntArray(NrOfDOFs);
    
    constrained = std::vector<bool>(NrOfDOFs,false);
    constrained_count = 0;
    
    kinematic_traversal = KDL::CoDyCo::Traversal();
    dynamic_traversal = KDL::CoDyCo::Traversal();
    
    measured_wrenches =  std::vector< KDL::Wrench >(NrOfFTSensors);
    
	v = std::vector<KDL::Twist>(NrOfLinks);
	a = std::vector<KDL::Twist>(NrOfLinks);
    
	f = std::vector<KDL::Wrench>(NrOfLinks); /**< wrench trasmitted by a link to the predecessor (note that predecessor definition depends on the selected dynamic base */
        
    f_ext = std::vector<KDL::Wrench>(NrOfLinks,KDL::Wrench::Zero());     
        
    //Create default kinematic traversal (if imu_names is wrong, creating the default one)   
    ret = tree_graph.compute_traversal(kinematic_traversal,imu_link_name);
    
    if( ret < 0 ) { std::cerr << "iDynTree constructor: imu_link_name not found" << std::endl; }
    assert(ret == 0);
    
    //Create default dynamics traversal (the dynamical base is the default one of the KDL::Tree)
    //Note that the selected dynamical base affect only the "classical" use case, when unkown external
    //wrenches are not estimated and are assume acting on the base. 
    //If the external forces are properly estimated, the base link for dynamic loop should not 
    //affect the results (i.e. 
    ret = tree_graph.compute_traversal(dynamic_traversal);
    assert(ret == 0);
    
    //iDynTreeContact
    ret = buildSubGraphStructure(joint_sensor_names);
    if( ret != 0 ) { std::cerr << "iDynTree constructor: ft sensor specified not found" << std::endl; }
    
    //building matrix and vectors for each subgraph
    A_contacts.resize(NrOfDynamicSubGraphs);
    b_contacts.resize(NrOfDynamicSubGraphs);
    
    b_contacts_subtree.resize(NrOfLinks);
    
    //end iDynTreeContact

    
    assert(ret == 0);
}

DynTree::~DynTree() { } ;

double DynTree::setAng(const double q_in, const int i)
{
	if (constrained[i]) {
		q(i) = (q_in<q_min(i)) ? q_min(i) : ((q_in>q_max(i)) ? q_max(i) : q_in);
	} else {
		q(i) = q_in;
	}
	return q(i);
}


//====================================
//
//      iDynTreeContact methods
//
//====================================

bool DynTree::isFTsensor(const std::string & joint_name, const std::vector<std::string> & ft_sensors) const
{
    if (std::find(ft_sensors.begin(), ft_sensors.end(), joint_name) != ft_sensors.end())
    {
        return true;
    }
    //else 
    {
        return false;
    }
}

int DynTree::buildSubGraphStructure(const std::vector<std::string> & ft_names)
{
    link2subgraph_index.resize(NrOfLinks,-1);
    link_is_subgraph_root.resize(NrOfLinks,false);
    link_FT_sensors.resize(NrOfLinks,std::vector<FTSensor *>(0));
    subgraph_index2root_link.resize(NrOfDynamicSubGraphs,-1);
    
    int next_id = 0;
    
    for(int i=0; i < (int)dynamic_traversal.order.size(); i++) {
        
        KDL::CoDyCo::LinkMap::const_iterator link_it = dynamic_traversal.order[i];
        int link_nmbr = link_it->link_nr; 
        
        if( i == 0 ) {
            
            //Starting with the dynamical base link, assign an index to the subgraph
            assert( dynamic_traversal.parent[link_nmbr] == tree_graph.getInvalidLinkIterator() );
            link2subgraph_index[link_nmbr] = next_id;
            
            //The dynamical base link is the root of its subgraph
            link_is_subgraph_root[link_nmbr] = true;
            subgraph_index2root_link[link2subgraph_index[link_nmbr]] = link_nmbr;
            
            next_id++;
            
        } else {
            //For every link, the subgraph is the same of its parent, unless it is connected to it by an FT sensor
            KDL::CoDyCo::LinkMap::const_iterator parent_it = dynamic_traversal.parent[link_it->link_nr];
            int parent_nmbr = parent_it->link_nr;
            
            if( isFTsensor(link_it->getAdjacentJoint(parent_it)->joint.getName(),ft_names) ) {
				//The FT sensor should be a fixed joint ? probably not
				//assert(link_it->getAdjacentJoint(parent_it)->joint.getType() == Joint::None);
				
                link2subgraph_index[link_nmbr] = next_id;
                
                //This link is a root of a dynamical subgraph, as its parent is in another subgraph
                link_is_subgraph_root[link_nmbr] = true;
                subgraph_index2root_link[link2subgraph_index[link_nmbr]] = link_nmbr;

                next_id++;
            } else {
                link2subgraph_index[link_nmbr] = link2subgraph_index[parent_nmbr];
                
                //This link is not a root of a dynamical subgraph
                link_is_subgraph_root[link_nmbr] = false;
            }
        }
    }
    
    //Building Force/Torque sensors data structures
    ft_list.clear();
    for(int i=0; i < (int)ft_names.size(); i++ ) {
		int parent_id = tree_graph.getJunction(ft_names[i])->parent->link_nr;
		int child_id = tree_graph.getJunction(ft_names[i])->child->link_nr;
		int sensor_id = i;
		ft_list.push_back(FTSensor(tree_graph,ft_names[i],parent_id,child_id,sensor_id));
		
		link_FT_sensors[parent_id].push_back(&(ft_list[i]));
		link_FT_sensors[child_id].push_back(&(ft_list[i]));
	}
        
    //The numbers of ids must be equal to the number of subgraphs
    if(next_id == NrOfDynamicSubGraphs) {
        return 0;
    } else {
		assert(false);
        return -1;
    }
    assert(false);
    return -1;
}

KDL::Wrench DynTree::getMeasuredWrench(int link_id)
{
	KDL::Wrench ret = KDL::Wrench::Zero();
	for(int i = 0; i < (int)link_FT_sensors[link_id].size(); i++ ) {
		ret += link_FT_sensors[link_id][i]->getWrenchExcertedOnSubGraph(link_id,measured_wrenches);
	}
	return ret;
}

void DynTree::buildAb_contacts()
{
	//First calculate the known terms b related to inertial, gravitational and 
	//measured F/T 
	
	for(int l=dynamic_traversal.order.size()-1; l>=0; l-- ) {  
            KDL::CoDyCo::LinkMap::const_iterator link_it = dynamic_traversal.order[l];
            int link_nmbr = link_it->link_nr;
            //Collect RigidBodyInertia and external forces
            KDL::RigidBodyInertia Ii= link_it->I;
            //This calculation should be done one time in forward kineamtic loop and stored \todo
            b_contacts_subtree[link_nmbr] = Ii*a[link_nmbr]+v[link_nmbr]*(Ii*v[link_nmbr]) - getMeasuredWrench(link_nmbr);
    }
    

    for(int l=dynamic_traversal.order.size()-1; l>=0; l-- ) {
            
		KDL::CoDyCo::LinkMap::const_iterator link_it = dynamic_traversal.order[l];
		int link_nmbr = link_it->link_nr;
		
		if( l != 0 ) {
			
			KDL::CoDyCo::LinkMap::const_iterator parent_it = dynamic_traversal.parent[link_nmbr];
			const int parent_nmbr = parent_it->link_nr;
			//If this link is a subgraph root, store the result, otherwise project it to the parent
			if( !isSubGraphRoot(link_nmbr) ) {
				double joint_pos;
				
				KDL::CoDyCo::JunctionMap::const_iterator joint_it = link_it->getAdjacentJoint(parent_it);
                if( joint_it->joint.getType() == KDL::Joint::None ) {
                    joint_pos = 0.0;
                } else {
                    joint_pos = q(link_it->getAdjacentJoint(parent_it)->q_nr);
                }
             
				
				b_contacts_subtree[parent_nmbr] += link_it->pose(parent_it,joint_pos)*b_contacts_subtree[link_nmbr]; 
			} else {
				b_contacts[getSubGraphIndex(link_nmbr)].setSubvector(0,KDLtoYarp(b_contacts_subtree[link_nmbr].torque));
				b_contacts[getSubGraphIndex(link_nmbr)].setSubvector(3,KDLtoYarp(b_contacts_subtree[link_nmbr].force));
			}
       }
	}
	
	//Then calculate the A and b related to unknown contacts
	
    iCub::skinDynLib::dynContactList::const_iterator it;
    
    std::vector<int> unknowns(NrOfDynamicSubGraphs,0);
    
    //Calculate the number of unknowns for each subgraph
    for(int sg=0; sg < NrOfDynamicSubGraphs; sg++ ) {
        for(it = contacts[sg].begin(); it!=contacts[sg].end(); it++)
        {
            if(it->isMomentKnown())
            {
                if(it->isForceDirectionKnown()) 
                {
                    unknowns[sg]++;     // 1 unknown (force module)
                }
                else
                {
                    unknowns[sg]+=3;    // 3 unknowns (force)
                }
            }
            else
            {
                unknowns[sg]+=6;        // 6 unknowns (force and moment)
            }
        }
    }
    
    
    for(int sg=0; sg < NrOfDynamicSubGraphs; sg++ ) {
        //Resize the A matrices
        A_contacts[sg] = yarp::sig::Matrix(6,unknowns[sg]);
        
        //Calculate A and b related to contacts
        int colInd = 0;
        for( it = contacts[sg].begin(); it!=contacts[sg].end(); it++)
        {
            //get link index
            int body_part = it->getBodyPart();
            int local_link_index = it->getLinkNumber();
            int link_contact_index = partition.getGlobalLinkIndex(body_part,local_link_index);
            
            //Subgraph information
            int subgraph_index = link2subgraph_index[link_contact_index];
            
            assert(subgraph_index == sg);
            
            int subgraph_root = subgraph_index2root_link[subgraph_index];
            
            //Get Frame transform between contact link and subgraph root
            //Inefficient but leads to cleaner code, if necessary can be improved 
            KDL::Frame H_root_link = getFrameLoop(tree_graph,q,dynamic_traversal,subgraph_root,link_contact_index);
            
            KDL::Vector COP;
            YarptoKDL(it->getCoP(),COP); 
            
            KDL::Frame H_link_contact = KDL::Frame(COP);
            KDL::Frame H_root_contact = H_root_link*H_link_contact;
            
            if(it->isForceDirectionKnown())
            {   
				// 1 UNKNOWN: FORCE MODULE
				yarp::sig::Matrix un(6,1);
				un.zero();
				un.setSubcol(it->getForceDirection(),3,0); // force direction unit vector
				yarp::sig::Matrix H_adj_root_contact = KDLtoYarp_wrench(H_root_contact);
				yarp::sig::Matrix col = H_adj_root_contact*un;
				A_contacts[sg].setSubmatrix(col,0,colInd);
                colInd += 1;
               
            }
            else
            {   
                if( it->isMomentKnown() ) {
                    // 3 UNKNOWNS: FORCE
                    A_contacts[sg].setSubmatrix(KDLtoYarp_wrench(H_root_contact).submatrix(0,5,3,5),0,colInd);
                    colInd += 3;
                    
                } else {                       
                    // 6 UNKNOWNS: FORCE AND MOMENT
                    A_contacts[sg].setSubmatrix(KDLtoYarp_wrench(H_root_contact),0,colInd);
                    colInd += 6;
                }
                
            }
        }
    }


}

void DynTree::store_contacts_results()
{
	//Make sure that the external forces are equal to zero before storing the results
	for(int l=dynamic_traversal.order.size()-1; l>=0; l-- ) {  
		f_ext[dynamic_traversal.order[l]->link_nr] = KDL::Wrench::Zero();
	}

	for(int sg=0; sg < NrOfDynamicSubGraphs; sg++ ) {
		unsigned int unknownInd = 0;
		iCub::skinDynLib::dynContactList::iterator it;
        for(it = contacts[sg].begin(); it!=contacts[sg].end(); it++)
        {

			//Store the result in dynContactList, for output
			if(it->isForceDirectionKnown()) {
				//1 UNKNOWN
				it->setForceModule( x_contacts[sg](unknownInd++));
			}
			else
			{
				if(it->isMomentKnown())
				{
					//3 UNKNOWN
					it->setForce(x_contacts[sg].subVector(unknownInd, unknownInd+2));
					unknownInd += 3;				
				} else {
					//6 UNKNOWN
					it->setMoment(x_contacts[sg].subVector(unknownInd, unknownInd+2));
					unknownInd += 3;
					it->setForce(x_contacts[sg].subVector(unknownInd, unknownInd+2));
					unknownInd += 3;

				}
			}
			
			//Store the results in f_ext, for RNEA dynamic loop
			KDL::Vector COP, force, moment;
			YarptoKDL(it->getCoP(),COP);
			KDL::Frame H_link_contact = KDL::Frame(COP);
			YarptoKDL(it->getForce(),force);
			YarptoKDL(it->getMoment(),moment);
			
			KDL::Wrench f_contact = KDL::Wrench(force,moment);
			//Get global link index from part ID and local index
			int link_nmbr = partition.getGlobalLinkIndex(it->getBodyPart(),it->getLinkNumber());
			f_ext[link_nmbr] = H_link_contact*f_contact;
        }
    }
}

//====================================
//
//      Set/Get methods
//
//====================================

yarp::sig::Vector DynTree::setAng(const yarp::sig::Vector & _q, const std::string & part_name) 
{
	yarp::sig::Vector ret_q = _q;
	
    if( part_name.length() ==  0 ) 
    { 
		//No part specified
		if( (int)_q.size() != NrOfDOFs ) { std::cerr << "setAng: Input vector has a wrong number of elements" << std::endl; return yarp::sig::Vector(0); } 
		if( constrained_count == 0 ) {
			//if all are not constrained, use a quicker way to copy
			YarptoKDL(_q,q);
		} else {
			for(int i =0; i < NrOfDOFs; i++ ){
				ret_q[i] = setAng(_q[i],i);
			}
		}
		
	} else { // if part_name.length > 0 
		const std::vector<int> & dof_ids = partition.getPartDOFIDs(part_name);
		if( dof_ids.size() != _q.size() ) { std::cerr << "setAng: Input vector has a wrong number of elements (or part_name wrong)" << std::endl; return yarp::sig::Vector(0); }
		for(int i = 0; i < (int)dof_ids.size(); i++ ) {
			ret_q[i] = setAng(_q[i],dof_ids[i]);
		}
	}
	return ret_q;
}

yarp::sig::Vector DynTree::getAng(const std::string & part_name) const
{    
	yarp::sig::Vector ret;
    if( part_name.length() == 0 ) 
    {
		KDLtoYarp(q,ret);
    } else {
		const std::vector<int> & dof_ids = partition.getPartDOFIDs(part_name);
		if( dof_ids.size() ==0  ) { std::cerr << "getAng: wrong part_name (or part with 0 DOFs)" << std::endl; return yarp::sig::Vector(0); }
		ret.resize(dof_ids.size());
		for(int i = 0; i < (int)dof_ids.size(); i++ ) {
			ret[i] = q(dof_ids[i]);
		}
	}
    return ret;
}

yarp::sig::Vector DynTree::setDAng(const yarp::sig::Vector & _q, const std::string & part_name)
{
    if( part_name.length() == 0 ) {
		if( (int)_q.size() != NrOfDOFs  ) { std::cerr << "setDAng: Input vector has a wrong number of elements" << std::endl; return yarp::sig::Vector(0); } 
		YarptoKDL(_q,dq);
	} 
	else 
	{
		const std::vector<int> & dof_ids = partition.getPartDOFIDs(part_name);
		if( dof_ids.size() != _q.size() ) { std::cerr << "setDAng: Input vector has a wrong number of elements (or part_name wrong)" << std::endl; return yarp::sig::Vector(0); }
		for(int i = 0; i < (int)dof_ids.size(); i++ ) {
			dq(dof_ids[i]) = _q[i];
		}
	}
	return _q;
}

yarp::sig::Vector DynTree::getDAng(const std::string & part_name) const
{
	yarp::sig::Vector ret;
    if( part_name.length() == 0 ) 
    {
		KDLtoYarp(dq,ret);
    } else {
		const std::vector<int> & dof_ids = partition.getPartDOFIDs(part_name);
		if( dof_ids.size() ==0  ) { std::cerr << "getDAng: wrong part_name (or part with 0 DOFs)" << std::endl; return yarp::sig::Vector(0); }
		ret.resize(dof_ids.size());
		for(int i = 0; i < (int)dof_ids.size(); i++ ) {
			ret[i] = dq(dof_ids[i]);
		}
	}
    return ret;

}

yarp::sig::Vector DynTree::setD2Ang(const yarp::sig::Vector & _q, const std::string & part_name)
{
    if( part_name.length() == 0 ) {
		if( (int)_q.size() != NrOfDOFs  ) { std::cerr << "setD2Ang: Input vector has a wrong number of elements" << std::endl; return yarp::sig::Vector(0); } 
		YarptoKDL(_q,ddq);
	} 
	else 
	{
		const std::vector<int> & dof_ids = partition.getPartDOFIDs(part_name);
		if( dof_ids.size() != _q.size() ) { std::cerr << "setD2Ang: Input vector has a wrong number of elements (or part_name wrong)" << std::endl; return yarp::sig::Vector(0); }
		for(int i = 0; i < (int)dof_ids.size(); i++ ) {
			ddq(dof_ids[i]) = _q[i];
		}
	}
	return _q;
}

yarp::sig::Vector DynTree::getD2Ang(const std::string & part_name) const
{
	yarp::sig::Vector ret;
    if( part_name.length() == 0 ) 
    {
		KDLtoYarp(ddq,ret);
    } else {
		const std::vector<int> & dof_ids = partition.getPartDOFIDs(part_name);
		if( dof_ids.size() ==0  ) { std::cerr << "getD2Ang: wrong part_name (or part with 0 DOFs)" << std::endl; return yarp::sig::Vector(0); }
		ret.resize(dof_ids.size());
		for(int i = 0; i < (int)dof_ids.size(); i++ ) {
			ret[i] = ddq(dof_ids[i]);
		}
	}
    return ret;
}

bool DynTree::setInertialMeasure(const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0)
{
    KDL::Vector w0_kdl, dw0_kdl, ddp0_kdl;
    YarptoKDL(w0,w0_kdl);
    YarptoKDL(dw0,dw0_kdl);
    YarptoKDL(ddp0,ddp0_kdl);
    imu_velocity.vel = KDL::Vector::Zero();
    imu_velocity.rot = w0_kdl;
    imu_acceleration.vel = ddp0_kdl;
    imu_acceleration.rot = dw0_kdl;
    return true;
}

bool DynTree::getInertialMeasure(yarp::sig::Vector &w0, yarp::sig::Vector &dw0, yarp::sig::Vector &ddp0) const
{
    //should care about returning the 3d acceleration instead of the spatial one? 
    //assuming the base linear velocity as 0, they are the same
    KDLtoYarp(imu_velocity.rot,w0);
    KDLtoYarp(imu_acceleration.vel,ddp0);
    KDLtoYarp(imu_acceleration.rot,dw0);
    return true;
}

bool DynTree::setSensorMeasurement(const int sensor_index, const yarp::sig::Vector &ftm)
{
    if( sensor_index < 0 || sensor_index > (int)measured_wrenches.size() ) { return false; }
    if( ftm.size() != 6 ) { return false; }
    YarptoKDL(ftm.subVector(0,2),measured_wrenches[sensor_index].force);
    YarptoKDL(ftm.subVector(3,5),measured_wrenches[sensor_index].torque);
    
    are_contact_estimated = false;
    
    return true;
}

bool DynTree::getSensorMeasurement(const int sensor_index, yarp::sig::Vector &ftm) const
{
    //\todo avoid unnecessary memory allocation
    yarp::sig::Vector force_yarp(3);
    yarp::sig::Vector torque_yarp(3);
    if( sensor_index < 0 || sensor_index > (int)measured_wrenches.size() ) { return false; }
    if( ftm.size() != 6 ) { ftm.resize(6); }
    KDLtoYarp(measured_wrenches[sensor_index].force,force_yarp);
    KDLtoYarp(measured_wrenches[sensor_index].torque,torque_yarp);
    ftm.setSubvector(0,force_yarp);
    ftm.setSubvector(3,torque_yarp);
    return true;
}

yarp::sig::Matrix DynTree::getPosition(const int link_index) const
{
	//\todo add that computation are updated checking
	return KDLtoYarp_position(X_dynamic_base[link_index]);
}

yarp::sig::Matrix DynTree::getPosition(const std::string & link_name) const
{
	KDL::CoDyCo::LinkMap::const_iterator link_it = tree_graph.getLink(link_name);
	if( link_it == tree_graph.getInvalidLinkIterator() ) { std::cerr << "DynTree::getPosition : link " << link_name << " not found" << std::endl; return Matrix(0,0); }
	//\todo add that computation are updated checking
	return getPosition(link_it->link_nr);
}

yarp::sig::Matrix DynTree::getPosition(const int first_link, const int second_link) const
{
	//\todo add that computation are update checking
	return KDLtoYarp_position(X_dynamic_base[first_link].Inverse()*X_dynamic_base[second_link]);
}

yarp::sig::Matrix DynTree::getPosition(const std::string & first_link_name, const std::string & second_link_name ) const
{
	KDL::CoDyCo::LinkMap::const_iterator first_link_it = tree_graph.getLink(first_link_name);
	if( first_link_it == tree_graph.getInvalidLinkIterator() ) { std::cerr << "DynTree::getPosition : link " << first_link_name << " not found" << std::endl; return Matrix(0,0); }
	KDL::CoDyCo::LinkMap::const_iterator second_link_it = tree_graph.getLink(second_link_name);
	if( second_link_it == tree_graph.getInvalidLinkIterator() ) { std::cerr << "DynTree::getPosition : link " << second_link_name << " not found" << std::endl; return Matrix(0,0); }	
	return getPosition(first_link_it->link_nr,second_link_it->link_nr);
}

yarp::sig::Vector DynTree::getVel(const int link_index) const
{
	if( link_index < 0 || link_index >= (int)tree_graph.getNrOfLinks() ) { std::cerr << "DynTree::getVel: link index " << link_index <<  " out of bounds" << std::endl; return yarp::sig::Vector(0); }
	yarp::sig::Vector ret(6), lin_vel(3), ang_vel(3);
	KDLtoYarp(v[link_index].vel,lin_vel);
	KDLtoYarp(v[link_index].rot,ang_vel);
	ret.setSubvector(0,lin_vel);
	ret.setSubvector(3,ang_vel);
	return ret;
}

yarp::sig::Vector DynTree::getVel(const std::string & link_name) const
{
	KDL::CoDyCo::LinkMap::const_iterator link_it = tree_graph.getLink(link_name);
	if( link_it == tree_graph.getInvalidLinkIterator() ) { std::cerr << "DynTree::getVel : link " << link_name << " not found" << std::endl; return Vector(0); }
	//\todo add that computation are updated checking
	return getVel(link_it->link_nr);
}

yarp::sig::Vector DynTree::getAcc(const int link_index) const
{
	if( link_index < 0 || link_index >= (int)tree_graph.getNrOfLinks() ) { std::cerr << "DynTree::getAcc: link index " << link_index <<  " out of bounds" << std::endl; return yarp::sig::Vector(0); }
	yarp::sig::Vector ret(6), classical_lin_acc(3), ang_acc(3);
	KDLtoYarp(a[link_index].vel+v[link_index].rot*v[link_index].vel,classical_lin_acc);
	KDLtoYarp(a[link_index].rot,ang_acc);
	ret.setSubvector(0,classical_lin_acc);
	ret.setSubvector(3,ang_acc);
	return ret;
}

yarp::sig::Vector DynTree::getAcc(const std::string & link_name) const
{
	KDL::CoDyCo::LinkMap::const_iterator link_it = tree_graph.getLink(link_name);
	if( link_it == tree_graph.getInvalidLinkIterator() ) { std::cerr << "DynTree::getAcc : link " << link_name << " not found" << std::endl; return Vector(0); }
	//\todo add that computation are updated checking
	return getAcc(link_it->link_nr);
}

    
yarp::sig::Vector DynTree::getTorques(const std::string & part_name) const
{
	if( part_name.length() == 0 ) {
		yarp::sig::Vector ret(NrOfDOFs);
		KDLtoYarp(torques,ret);
		return ret;
	} else {
		const std::vector<int> & dof_ids = partition.getPartDOFIDs(part_name);
		if( dof_ids.size() ==0  ) { std::cerr << "getTorques: wrong part_name (or part with 0 DOFs)" << std::endl; return yarp::sig::Vector(0); }
		yarp::sig::Vector ret(dof_ids.size());
		for(int i = 0; i < (int)dof_ids.size(); i++ ) {
			ret[i] = torques(dof_ids[i]);
		}
		return ret;
	}
}
    
bool DynTree::setContacts(const iCub::skinDynLib::dynContactList & contacts_list)
{
    for(int sg = 0; sg < NrOfDynamicSubGraphs; sg++ ) {
        contacts[sg].resize(0);
    }
    
    //Separate unknown contacts depending on their subgraph
    for(iCub::skinDynLib::dynContactList::const_iterator it = contacts_list.begin();
            it != contacts_list.end(); it++ ) 
    {
        //get link index
        int body_part = it->getBodyPart();
        int local_link_index = it->getLinkNumber();
        int link_contact_index = partition.getGlobalLinkIndex(body_part,local_link_index);
        
        int subgraph_id = getSubGraphIndex(link_contact_index);
        
        contacts[subgraph_id].push_back(*it);
    }
    
    are_contact_estimated = false;
    
    return true; 
}
    
const iCub::skinDynLib::dynContactList DynTree::getContacts() const
{
    iCub::skinDynLib::dynContactList all_contacts(0);
    
   
    for(int sg = 0; sg < NrOfDynamicSubGraphs; sg++ )
    {
        all_contacts.insert(all_contacts.end(),contacts[sg].begin(),contacts[sg].end());
    }
    
    return all_contacts;
}
    

//====================================
//
//      Computation methods
//
//====================================
bool DynTree::computePositions()
{
	if(X_dynamic_base.size() != tree_graph.getNrOfLinks()) { X_dynamic_base.resize(tree_graph.getNrOfLinks()); }
	if( getFramesLoop(tree_graph,q,dynamic_traversal,X_dynamic_base) == 0 ) return true;
	//else
	 return false;  
}

bool DynTree::kinematicRNEA()
{
    int ret;
    ret = rneaKinematicLoop(tree_graph,q,dq,ddq,kinematic_traversal,imu_velocity,imu_acceleration,v,a);
    
    are_contact_estimated = false;
    
    if( ret < 0 ) return false;
    //else
    return true;
}

bool DynTree::estimateContactForces()
{
    double tol = 1e-7; /**< value extracted from old iDynContact */
    buildAb_contacts();
    for(int i=0; i < NrOfDynamicSubGraphs; i++ ) {
        x_contacts[i] = yarp::math::pinv(A_contacts[i],tol)*b_contacts[i];
    }
    store_contacts_results();
    are_contact_estimated = true;
    return true;
}
    
bool DynTree::dynamicRNEA()
{
    int ret;
    KDL::Wrench base_force;
    ret = rneaDynamicLoop(tree_graph,q,dynamic_traversal,v,a,f_ext,f,torques,base_force);
    //Check base force: if estimate contact was called, it should be zero
    if( are_contact_estimated == true ) {
		//If the force were estimated wright
		assert( base_force.force.Norm() < 1e-5 );
		assert( base_force.torque.Norm() < 1e-5 );
		//Note: this (that no residual appears happens only for the proper selection of the provided dynContactList
		for(int i=0; i < NrOfFTSensors; i++ ) {
			KDL::Wrench residual = measured_wrenches[i] - ft_list[i].getH_child_sensor().Inverse(f[ft_list[i].getChild()]);
			assert( residual.force.Norm() < 1e-5 );
			assert( residual.torque.Norm() < 1e-5 );
		}
		
		
	} else {
		//In case contacts forces where not estimated, the sensor values have
		//to be calculated from the RNEA
		for(int i=0; i < NrOfFTSensors; i++ ) {
			//Todo add case that the force/wrench is the one of the parent ?
			#ifndef NDEBUG
			std::cerr << "Project sensor " << i << "from link " << ft_list[i].getChild() << " to sensor " << std::endl;
			std::cerr << "Original force " << KDLtoYarp(f[ft_list[i].getChild()].force).toString() << std::endl;
			#endif
			measured_wrenches[i] = ft_list[i].getH_child_sensor().Inverse(f[ft_list[i].getChild()]);
		}
	}
    if( ret < 0 ) return false;
    return true;
}

////////////////////////////////////////////////////////////////////////
////// COM related methods
////////////////////////////////////////////////////////////////////////
    bool DynTree::computeCOM() 
    {
		return false;
	}
    
    bool DynTree::computeCOMjacobian()
    {
		return false;
	}

    yarp::sig::Vector DynTree::getCOM(const std::string & part_name) const
    {
		return yarp::sig::Vector(0);
	}
    
    bool DynTree::getCOMJacobian(const yarp::sig::Matrix & jac, const std::string & part_name) const
    {
		return false;
	}


}
}
