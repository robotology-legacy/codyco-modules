#ifndef FLOATING_BASE_H_
#define FLOATING_BASE_H_

#include <iostream>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Property.h>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>


/**
 *  This class estimate the floating base attitude within QuaternionEKF.
 */

namespace wholeBodyEstimator
{
    class floatingBase
    {
    private:
        wbi::wholeBodyInterface * m_robot;
        yarp::os::Property        m_module_params;
        MatrixWrapper::Matrix     m_rot_from_ft_to_acc;
        yarp::sig::Vector         m_q;

    public:

        floatingBase();
        virtual ~floatingBase();

        /**
         *  This method should instantiate the yarpWholeBodyInterface and initialize it.
         *
         *  @return true if everything is successful.
         */
        bool configure( yarp::os::ResourceFinder &rf, MatrixWrapper::Matrix rot_from_ft_to_acc );
        
        /**
         *  Computes the rotation matrix from the floating base to the world reference frame. The world reference frame in this module is given by the initial pose of the accelerometer's reference frame on the right foot of the robot.
         *
         *  @param rot_from_world_to_sensor       Rotation matrix from world to sensor. This should be the result of the estimation done by QuaternionEKF.
         *  @param rot_from_floatingBase_to_world Output matrix providing the rotation from floating base to the world reference frame.
         *
         *  @return true if successsful, false otherwise.
         */
        bool compute_Rot_from_floatingBase_to_world( MatrixWrapper::Matrix rot_from_world_to_sensor, MatrixWrapper::Matrix &rot_from_floatingBase_to_world );
        
        
        /**
         *  Retrieves whole body interface options from the Resource Finder as well as the modle joints used to instantiate the yarpWholeBodyInterface.
         *
         *  @param rf                      Resource finder as passed by the module.
         *  @param RobotDynamicModelJoints Robot model joints list (output).
         *  @param yarpWbiOptions          yarpWholeBodyInterface options (output).
         *
         *  @return True if successful, false otherwise.
         */
        bool getWbiOptionsAndModelJoints(yarp::os::ResourceFinder &rf,
                                         wbi::IDList &RobotDynamicModelJoints,
                                         yarp::os::Property &yarpWbiOptions);

    };

}

#endif
