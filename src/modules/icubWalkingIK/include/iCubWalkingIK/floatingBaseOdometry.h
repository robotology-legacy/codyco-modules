#ifndef FLOATING_BASE_ODOMETRY_H
#define FLOATING_BASE_ODOMETRY_H

#include <iDynTree/Estimation/simpleLeggedOdometry.h>
#include <yarpWholeBodyInterface/yarpWholeBodyModel.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/LogStream.h>
#include <iDynTree/Estimation/robotStatus.h>
#include <iCub/iDynTree/yarp_kdl.h>

class floatingBaseOdometry {
private:
    yarpWbi::yarpWholeBodyModel * m_wbm;
    iDynTree::simpleLeggedOdometry m_floatingBase;
    iCub::iDynTree::DynTree * m_robot_model;
    std::string m_current_fixed_link_name;
    int m_floating_base_frame_index;
    yarp::sig::Matrix m_world_H_floatingBase;
    iDynTree::RobotJointStatus * m_joint_status;
public:
    floatingBaseOdometry(yarpWbi::yarpWholeBodyModel * wbm);
    virtual ~floatingBaseOdometry();
    bool init(std::string initial_world_frame_position = "l_sole",
              std::string initial_fixed_link = "r_sole",
              std::string floating_base_frame_index = "root_link");
    void update(double* joints_configuration);
    void get_world_H_floatingbase(wbi::Frame &m);
};

#endif