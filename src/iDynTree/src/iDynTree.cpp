/*
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 */

#include "iCub/iDynTree/iDynTree.hpp"

#include "iCub/iDynTree/yarp_kdl.h"


#include <iostream>


/**
 * 
 * \todo implement range checking and part handling
 * 
 */ 
 

using namespace iCub::iDynTree;

iDynTree::iDynTree(const KDL::Tree & _tree, 
                   const std::vector<std::string> & joint_sensor_names,
                   const std::string & imu_link_name,
                   TreeSerialization serialization): tree(_tree)
{
    //If serialization is not consistent (as if no serialization is given), use the default one 
    if(!serialization.is_consistent(tree)) {
        serialization = TreeSerialization(tree);
    }
    
    //Setting useful constants
    NrOfDOFs = tree.getNrOfJoints();
    NrOfLinks = tree.getNrOfSegments();
    NrOfSensors = joint_sensor_names.size();
    NrOfDynamicSubTrees = NrOfSensors + 1;
    
    q = KDL::JntArray(NrOfDOFs);
    dq = KDL::JntArray(NrOfDOFs);
    ddq = KDL::JntArray(NrOfDOFs);
    
    measured_wrenches =  std::vector< KDL::Wrench >(NrOfSensors);
    
    X = std::vector<Frame>(NrOfLinks);
    S = std::vector<Twist>(NrOfDOFs);
	v = std::vector<Twist>(NrOfLinks);
	a = std::vector<Twist>(NrOfLinks);
	f = std::vector<Wrench>(NrOfLinks);
    
    
    //For the kinematic, the serialization is similar to a normal solver,
    //with the difference of using the imu link as the root one
    std::vector< int > children_root;
    std::vector< std::vector< int > > children;
    
    if( !serialization.serialize(tree,children_root,children,kinematic_parent,
                            kinematic_link2joint,kinematic_visit_order,segments_vector,imu_link_name) ) {
        std::cerr << "Kinematic serialization failed" << std::endl; return; 
    }
    
    //For dynamics
    
    
    
}

iDynTree::~iDynTree() { } ;


yarp::sig::Vector iDynTree::setAng(const yarp::sig::Vector & _q, const std::string & part_name) 
{
    if( part_name.length() > 0 ) { std::cerr << "Part handling not implemented yet!" << endl; }
    if( _q.size() != NrOfDOFs ) { std::cerr << "setAng: Input vector has a wrong number of elements" << endl; return yarp::sig::Vector(0); } 
    YarptoKDL(_q,q);
    return _q;
}

yarp::sig::Vector iDynTree::getAng(const std::string & part_name)
{
    if( part_name.length() > 0 ) { std::cerr << "Part handling not implemented yet!" << endl; }
    yarp::sig::Vector ret;
    KDLtoYarp(q,ret);
    return ret;
}

yarp::sig::Vector iDynTree::setDAng(const yarp::sig::Vector & _q, const std::string & part_name)
{
    if( part_name.length() > 0 ) { std::cerr << "Part handling not implemented yet!" << endl; }
    if( _q.size() != NrOfDOFs  ) { std::cerr << "setDAng: Input vector has a wrong number of elements" << endl; return yarp::sig::Vector(0); } 
    YarptoKDL(_q,dq);
    return _q;
}

yarp::sig::Vector iDynTree::getDAng(const std::string & part_name)
{
    if( part_name.length() > 0 ) { std::cerr << "Part handling not implemented yet!" << endl; }
    if( part_name.length() > 0 ) { std::cerr << "Part handling not implemented yet!" << endl; }
    yarp::sig::Vector ret;
    KDLtoYarp(dq,ret);
    return ret;

}

yarp::sig::Vector iDynTree::setD2Ang(const yarp::sig::Vector & _q, const std::string & part_name)
{
    if( part_name.length() > 0 ) { std::cerr << "Part handling not implemented yet!" << endl; }
    if( _q.size() != NrOfDOFs ) { std::cerr << "setD2Ang: Input vector has a wrong number of elements" << endl; return yarp::sig::Vector(0); } 
    YarptoKDL(_q,ddq);
    return _q;
}

yarp::sig::Vector iDynTree::getD2Ang(const std::string & part_name)
{
    if( part_name.length() > 0 ) { std::cerr << "Part handling not implemented yet!" << endl; }
    yarp::sig::Vector ret;
    KDLtoYarp(ddq,ret);
    return ret;
}

bool iDynTree::setInertialMeasure(const yarp::sig::Vector &w0, const yarp::sig::Vector &dw0, const yarp::sig::Vector &ddp0)
{
    KDL::Vector w0_kdl, dw0_kdl, ddp0_kdl;
    YarptoKDL(w0,w0_kdl);
    YarptoKDL(dw0,dw0_kdl);
    YarptoKDL(ddp0,ddp0_kdl);
    base_velocity.vel = KDL::Vector::Zero();
    base_velocity.rot = w0_kdl;
    base_acceleration.vel = ddp0_kdl;
    base_acceleration.rot = dw0;
    return true;
}

bool iDynTree::getInertialMeasure(yarp::sig::Vector &w0, yarp::sig::Vector &dw0, yarp::sig::Vector &ddp0)
{
    //should care about returning the 3d acceleration instead of the spatial one? 
    //assuming 0 base linear velocities they are the same
    KDLtoYarp(base_velocity.rot,w0);
    KDLtoYarp(base_acceleration.vel,ddp0);
    KDLtoYarp(base_acceleration.rot,dw0);
    return true;
}

bool iDynTree::setSensorMeasurement(const int sensor_index, const yarp::sig::Vector &ftm)
{
    if( sensor_index < 0 || sensor_index > measured_wrenches.size() ) { return false; }
    if( ftm.size() != 6 ) { return false; }
    YarptoKDL(ftm.subVector(0,2),measured_wrenches[sensor_index].force);
    YarptoKDL(ftm.subVector(3,5),measured_wrenches[sensor_index].torque);
    return true;
}

bool iDynTree::getSensorMeasurement(const int sensor_index, yarp::sig::Vector &ftm)
{
    //\todo avoid unnecessary memory allocation
    yarp::sig::Vector force_yarp(3);
    yarp::sig::Vector torque_yarp(3);
    if( sensor_index < 0 || sensor_index > measured_wrenches.size() ) { return false; }
    if( ftm.size() != 6 ) { ftm.resize(6); }
    KDLtoYarp(measured_wrenches[sensor_index].force,force_yarp);
    KDLtoYarp(measured_wrenches[sensor_index].torque,torque_yarp);
    ftm.setSubvector(0,force_yarp);
    ftm.setSubvector(2,torque_yarp)
    return true;
}

bool iDynTree::kinematicRNEA()
{
    int ret;
    ret = rneaKinematicLoop(q,dq,ddq,base_velocity,base_acceleration,
                            kinematic_visit_order,segments_vector,kinematic_parent,kinematic_link2joint,
                            kinematic_X,kinematic_S,v,a);
    if( ret != 0 ) return false;
    return true;
}

bool iDynTree::estimateContactForces()
{
    
}
    
bool iDynTree::dynamicRNEA()
{
    for( int i = 0; i < NrOfDynamicSubTrees; i++ ) {
        //solve dynamics for each subtree (using contacts)
    }
    return true;
}
    
yarp::sig::Vector iDynTree::getTorques(const std::string & part_name="") const
{
    yarp::sig::Vector ret(NrOfDOFs);
    KDLtoYarp(torques,ret);
    return ret;
}
    
bool setContacts(const iCub::skinDynLib::dynContactList & contacts_list)
{
    contacts = contacts_list;
    return true; 
}
    
const iCub::skinDynLib::dynContactList& getContacts() const
{
    return contacts;
}
    
