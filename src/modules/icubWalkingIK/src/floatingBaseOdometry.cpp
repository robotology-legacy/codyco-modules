#include "floatingBaseOdometry.h"

using namespace yarpWbi;

floatingBaseOdometry::floatingBaseOdometry(yarpWholeBodyModel * wbm) :
m_wbm(wbm)
{
    // Here we assume that wbm has been correctly initialized.
    m_robot_model = m_wbm->getRobotModel();
}

floatingBaseOdometry::~floatingBaseOdometry(){
    if ( m_joint_status )
    {
        delete m_joint_status;
        m_joint_status = 0;
    }
}

bool floatingBaseOdometry::init(std::string initial_world_frame_position,
                                std::string initial_fixed_link,
                                std::string floating_base_frame_index){
    bool ok;
//    std::string initial_world_frame_position = "l_sole";
//    std::string initial_fixed_link = "r_sole";
//    std::string floating_base_frame_index = "root";
    KDL::CoDyCo::UndirectedTree undirected_tree = this->m_robot_model->getKDLUndirectedTree();
    ok = m_floatingBase.init(undirected_tree,
                             initial_world_frame_position,
                             initial_fixed_link);
    //FIXME: This could be done also by init in simpleLeggedOdometry
    this->m_current_fixed_link_name = initial_fixed_link;
    this->m_floating_base_frame_index = m_floatingBase.getDynTree().getFrameIndex(floating_base_frame_index);
    
    ok = ok && (this->m_floating_base_frame_index >= 0 &&
                this->m_floating_base_frame_index < m_floatingBase.getDynTree().getNrOfFrames());
    
    if( !ok )
    {
        yError() << "floatingBaseOdometry initialization failed!";
        return false;
    }
    
    m_world_H_floatingBase.resize(4, 4);
    m_joint_status = new iDynTree::RobotJointStatus;
    m_joint_status->setNrOfDOFs(m_robot_model->getNrOfDOFs());
    m_joint_status->zero();
//    yarp::sig::Vector q_wbm_zero(m_wbm->getDoFs());
//    yarp::sig::Vector q_dyntree_zero;
//    m_wbm->convertQ(q_wbm_zero.data(), q_dyntree_zero);
    m_robot_model->setAng(m_joint_status->getJointPosYARP());
    
    return ok;

}

void floatingBaseOdometry::update(double* q_wbm, bool switch_foot=false){
    // Read robot status
//    double * q_dyntree = m_joint_status->getJointPosKDL().data.data();
    if (switch_foot)
        if ( !changeFixedFoot() )
        {
            yError("[floatingBaseOdometry:update()] Could not change fixed frame");
        }
    yarp::sig::Vector q_dyntree(m_robot_model->getNrOfDOFs());
    m_wbm->convertQ(q_wbm, q_dyntree);
    m_joint_status->setJointPosYARP(q_dyntree);
    // Update yarp vectors
    m_joint_status->updateYarpBuffers();
    // Read joint positions, velocities and accelerations.
    m_floatingBase.setJointsState(m_joint_status->getJointPosKDL(),
                                  m_joint_status->getJointVelKDL(),
                                  m_joint_status->getJointAccKDL());
    // Get floating base position in the world
    KDL::Frame world_H_floatingbase_kdl = m_floatingBase.getWorldFrameTransform(this->m_floating_base_frame_index);
    // Transform floating base position in YARP format
    KDLtoYarp_position(world_H_floatingbase_kdl, this->m_world_H_floatingBase);
    
    std::cerr << "world_H_floatingbase: \n" << m_world_H_floatingBase.toString().c_str() << std::endl;
    
}

void floatingBaseOdometry::get_world_H_floatingbase(wbi::Frame &H) {
    H.set4x4Matrix(m_world_H_floatingBase.data());
}

bool floatingBaseOdometry::changeFixedFoot(){
    return m_floatingBase.changeFixedFoot();
}

