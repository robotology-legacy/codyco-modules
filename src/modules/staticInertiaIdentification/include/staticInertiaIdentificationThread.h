/*
 * Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia - Italian Institute of Technology
 * Author: Silvio Traversaro
 * email:  silvio.traversaro@iit.it
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
*/

#ifndef STATIC_INERTIA_IDENTIFICATION_THREAD
#define STATIC_INERTIA_IDENTIFICATION_THREAD

#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <vector>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/Property.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/skinDynLib/skinContactList.h>

#include <wbi/wbi.h>
#include <wbiIcub/wholeBodyInterfaceIcub.h>

#include <kdl_codyco/regressors/dynamicRegressorGenerator.hpp>
#include <kdl_codyco/regressors/dataset/DynamicSample.hpp>

#include "parameterSVDLinearEstimator.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::skinDynLib;
using namespace std;
using namespace wbi;
using namespace Eigen;


/**
 *
  */
class staticInertiaIdentificationThread: public RateThread
{
    string              name;
    string              robotName;

    yarp::os::Mutex run_mutex;

    enum { ESTIMATING, NOT_ESTIMATING } thread_state;

    wbiIcub::icubWholeBodyStatesLocal *estimator;

    int                 printCountdown;         // every time this is 0 (i.e. every print_period ms) print stuff
    double              printPeriod;

    iCub::iDynTree::iCubTree_version_tag icub_version;
    iCub::iDynTree::iCubTree icub_tree_model;

    KDL::CoDyCo::Regressors::DynamicRegressorGenerator * icub_regressor_generator;
    Eigen::MatrixXd identificable_subspace_basis;

    parameterSVDLinearEstimator parameter_estimator;

    yarp::os::Property opts;

    std::vector< std::vector< std::string > > list_of_subtrees;
    std::vector< std::string > subtrees_names;

    KDL::CoDyCo::Regressors::DynamicSample * robot_status;
    KDL::JntArray last_joint_position_used_for_identification;


    yarp::sig::Vector wbi_imu;

    //Buffers
    Eigen::VectorXd ft_data_buffer;
    Eigen::VectorXd singular_value_buffer;
    Eigen::MatrixXd singular_vectors_buffer;

    Eigen::VectorXd known_terms;
    Eigen::MatrixXd regressor;
    Eigen::MatrixXd base_regressor;

    //Counters
    int poseCount;
    int currentPoseSampleCount;
    int maxNrOfSamplesForPose;

    //Flags
    bool is_static_sample;
    bool is_sample_a_new_pose;
    bool is_external_contact_sample;
    bool estimation_disabled_for_external_contact;

    //Timestamps
    double timestamp_of_last_external_contact;

    //Threshold
    double velocity_static_threshold;
    double position_norm_threshold;

    //Ports
    BufferedPort<iCub::skinDynLib::skinContactList> *port_skin_contacts;

    bool configureRegressorGenerator();

    bool configureGeneratedRegressor();

    bool configureParameterEstimator();

    void printStatus();

    bool isSampleStatic(const KDL::CoDyCo::Regressors::DynamicSample & sample);

    bool isSamplePositionDifferent(const KDL::CoDyCo::Regressors::DynamicSample & sample,
                                   const KDL::JntArray & previous_position);

    bool isSampleCorruptedByExternalContact();

    bool useCurrentSampleForIdentification();

    void closePort(Contactable *_port);


public:

    staticInertiaIdentificationThread(string _name,
                                      string _robotName,
                                      int _period,
                                      wbiIcub::icubWholeBodyStatesLocal *_wbi,
                                      const iCub::iDynTree::iCubTree_version_tag icub_version,
                                      yarp::os::Property opts,
                                      bool autoconnect);

    bool threadInit();
    void run();
    void threadRelease();

    bool startEstimation();
    bool stopEstimation();

    bool saveURDF(const std::string & urdf_filename, const std::string & robotName);


};


#endif
