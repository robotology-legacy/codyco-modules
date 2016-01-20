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
        
        
        bool compute_Rot_from_floatingBase_to_world( MatrixWrapper::Matrix rot_from_world_to_sensor, MatrixWrapper::Matrix &rot_from_floatingBase_to_world );
        
        
        bool getWbiOptionsAndModelJoints(yarp::os::ResourceFinder &rf,
                                         wbi::IDList &RobotDynamicModelJoints,
                                         yarp::os::Property &yarpWbiOptions);

    };

}

#endif
