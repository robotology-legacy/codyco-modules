
#include "SingleLeggedOdometry.h"

#include "kdl/frames_io.hpp"

SingleLeggedOdometry::SingleLeggedOdometry():
odometry_model(0),
current_fixed_link_id(-1),
world_H_fixed()
{
    
}

SingleLeggedOdometry::~SingleLeggedOdometry()
{
    if( odometry_model )
    {
        delete odometry_model;
        odometry_model=0;
    }
}

bool SingleLeggedOdometry::init(KDL::CoDyCo::UndirectedTree & undirected_tree,
                                const std::string& initial_world_frame_position,
                                const std::string& initial_fixed_link)
{
    int initial_world_frame_position_index = undirected_tree.getLink(initial_world_frame_position)->getLinkIndex();
    int initial_fixed_link_index = undirected_tree.getLink(initial_fixed_link)->getLinkIndex();;
    if( initial_fixed_link_index < 0 ||
       initial_world_frame_position_index < 0 )
    {
        return false;
    }
    
    return init(undirected_tree,initial_world_frame_position_index,initial_fixed_link_index);
}

bool SingleLeggedOdometry::init(KDL::CoDyCo::UndirectedTree & undirected_tree,
                                const int initial_world_frame_position_index,
                                const int initial_fixed_link_index)
{
    if( odometry_model )
    {
        delete odometry_model;
        odometry_model=0;
    }
    
    odometry_model = new iCub::iDynTree::DynTree(undirected_tree.getTree(),undirected_tree.getSerialization());
    bool ok = reset(initial_world_frame_position_index,initial_fixed_link_index);
    return ok;
}

bool SingleLeggedOdometry::reset(const std::string& initial_world_frame_position, const std::string& initial_fixed_link)
{
    int initial_world_frame_position_index = odometry_model->getLinkIndex(initial_world_frame_position);
    int initial_fixed_link_index = odometry_model->getLinkIndex(initial_fixed_link);
    if( initial_fixed_link_index < 0 ||
       initial_world_frame_position_index < 0 )
    {
        return false;
    }
    
    return reset(initial_world_frame_position_index,initial_fixed_link_index);
}

bool SingleLeggedOdometry::reset(const int initial_world_frame_position_index, const int initial_fixed_link_index)
{
    current_fixed_link_id = initial_fixed_link_index;
    world_H_fixed = odometry_model->getPositionKDL(initial_world_frame_position_index,initial_fixed_link_index);
    return true;
}

bool SingleLeggedOdometry::changeFixedLink(const std::string& new_fixed_link_name)
{
    int new_fixed_link_id = odometry_model->getLinkIndex(new_fixed_link_name);
    
    if( new_fixed_link_id < 0 )
    {
        return false;
    }
    
    return changeFixedLink(new_fixed_link_id);
}


bool SingleLeggedOdometry::changeFixedLink(const int& new_fixed_link_id)
{
    int old_fixed_link_id = this->current_fixed_link_id;
    KDL::Frame world_H_old_fixed = this->world_H_fixed;
    KDL::Frame old_fixed_H_new_fixed = odometry_model->getPositionKDL(old_fixed_link_id,new_fixed_link_id);
    this->world_H_fixed = world_H_old_fixed*old_fixed_H_new_fixed;
    this->current_fixed_link_id = new_fixed_link_id;
    return true;
}

std::string SingleLeggedOdometry::getCurrentFixedLink()
{
    std::string ret_string;
    odometry_model->getLinkName(this->current_fixed_link_id,ret_string);
    return ret_string;
}

KDL::Frame SingleLeggedOdometry::getWorldFrameTransform(const int frame_index)
{
    KDL::Frame fixed_H_frame = odometry_model->getPositionKDL(this->current_fixed_link_id,frame_index);
    return this->world_H_fixed*fixed_H_frame;
}

bool SingleLeggedOdometry::setJointsState(const KDL::JntArray& qj,
                                          const KDL::JntArray& dqj,
                                          const KDL::JntArray& ddqj)
{
    if( qj.rows() != odometry_model->getNrOfDOFs()  ||
       dqj.rows() != odometry_model->getNrOfDOFs()  ||
       ddqj.rows() != odometry_model->getNrOfDOFs() )
    {
        return false;
    }
    
    bool ok = true ;
    
    ok = ok && odometry_model->setAngKDL(qj);
    ok = ok && odometry_model->setDAngKDL(dqj);
    ok = ok && odometry_model->setD2AngKDL(ddqj);
    
    //Update also the floating base position, given this new joint positions
    KDL::Frame world_H_base = this->getWorldFrameTransform(odometry_model->getFloatingBaseLink());
    ok = ok && odometry_model->setWorldBasePoseKDL(world_H_base);
    
    return ok;
}

const iCub::iDynTree::DynTree & SingleLeggedOdometry::getDynTree()
{
    return *odometry_model;
}

bool SingleLeggedOdometry::init(yarp::os::ResourceFinder &rf)
{
    bool ret = false;
    return ret;
}

void SingleLeggedOdometry::run()
{
    
}

void SingleLeggedOdometry::release()
{
    
}