#include "floatingBase.h"


namespace wholeBodyEstimator
{
    floatingBase::floatingBase()
    {

    }

    floatingBase::~floatingBase()
    {
        if (m_robot)
        {
            yInfo("[floatingBase::floatingBase] Am I closing the interface?");
            m_robot->close();
            delete m_robot;
            m_robot = 0;
        }
    }

    bool floatingBase::configure(yarp::os::ResourceFinder &rf, MatrixWrapper::Matrix rot_from_ft_to_acc, wbi::iWholeBodySensors *wbs)
    {
        /**
         *  Copy pointer to iWholeBodySensors
         */
        m_wbs = wbs;
        
        /**
         *  Updating rotation matrix from FT sensor to accelerometer
         */
        m_rot_from_ft_to_acc = rot_from_ft_to_acc;

        /**
         *  Get module parameters of interest
         *
         */
        std::string module_name;
        int period;

        if (!rf.check("module_parameters"))
        {
            yError("Group module_parameters was not specified in the configuration file of this module. Please fix it and try again.");
            return false;
        } else {
            m_module_params.fromString(rf.findGroup("module_parameters").tail().toString());
            yInfo(" [floatingBase::configure] module_parameters group contents are: %s ", m_module_params.toString().c_str());

            period = m_module_params.find("period").asInt();
            module_name = m_module_params.find("name").asString();
        }

        /**
         *  Instantiate a yarpWholeBodyStates object
         */
        // Retrieve wbi options and model joints
        wbi::IDList RobotDynamicModelJoints;
        yarp::os::Property wbiProperties;
        getWbiOptionsAndModelJoints(rf, RobotDynamicModelJoints, wbiProperties);

        yInfo("[floatingBase::configure] wbiProperties passed were: %s ", wbiProperties.toString().c_str());
        yInfo("[floatingBase::configure] Robot DOF: %d ", RobotDynamicModelJoints.size());

        // Create reference to wbi
        m_robot = new yarpWbi::yarpWholeBodyModel(module_name.c_str(), wbiProperties);
        if ( !m_robot )
        {
            yError("[floatingBase::configure] Could not create wbi object");
            return false;
        }

        // Add joints
        m_robot->addJoints(RobotDynamicModelJoints);

        /**
         *  Initialize the yarpWholeBodyModel object
         */
        if ( !m_robot->init() )
        {
            yError("[floatingBase::configure] Could not initialize wbi");
            return false;
        }

        yInfo("[floatingBase::config] yarpWholeBodyModel was initialized correctly by floatingBase)");

        /**
         * Resizing private variables of interest
         */
        yInfo("[floatingBase::config] RobotMainJoints is: %s", RobotDynamicModelJoints.toString().c_str());
        unsigned int actuatedDOF = RobotDynamicModelJoints.size();
        yInfo("[floatingBase::config] actuatedDOF %i ", actuatedDOF);
        m_q.resize(actuatedDOF, 0.0);

        return true;
    }


    bool floatingBase::compute_Rot_from_floatingBase_to_world( MatrixWrapper::Matrix rot_from_world_to_sensor,
                                                               MatrixWrapper::Matrix &rot_from_floatingBase_to_world)
    {
        /**
         *  Compute rot_from_FT_to_floatingBase
         */
        // Retrieve joint angles (rad)
        m_wbs->readSensors(wbi::SENSOR_ENCODER_POS, m_q.data());
        //m_robot->getEstimates(wbi::ESTIMATE_JOINT_POS, m_q.data());
//        yInfo( "[floatingBase::compute_Rot_from_floatingBase_to_world()] %s ", m_q.toString().c_str() );

        // rot_from_foot_to_floating_base
        wbi::Frame H;
        // This is by default the identity, meaning that H will be expressed in the floating base.
        wbi::Frame xBase;
        // Compute rot_from_FT_to_floatingBase
        std::string linkName = "r_sole";
        int linkId;
        m_robot->getFrameList().idToIndex(linkName.c_str(), linkId);
        m_robot->computeH(m_q.data(), xBase, linkId, H);
        //std::cerr << "Rotation from r_sole to base is: " << std::endl << H.toString().c_str() << std::endl;
        // rot_from_acc_to_floating_base
        wbi::Rotation rot_from_foot_to_floating_base = H.R;
        //TODO: This has been hardcoded for the right foot!!! but this method should ask for the desired foot
        wbi::Rotation rot_from_FT_to_foot(1, 0, 0, 0, -1, 0, 0, 0 , -1);
        //FIXME: I should instead copy the values in the private variable m_rot_from_ft_to_acc
        // This should rot_fom_FT_to_foot.inverse() (or transpose) but it would be the same for this particular rotation mat.
        wbi::Rotation rot_from_acc_to_FT = rot_from_FT_to_foot;
        wbi::Rotation rot_from_acc_to_foot = rot_from_FT_to_foot * rot_from_acc_to_FT;
        // Copute rot_from_acc_to_floating_base
        wbi::Rotation rot_from_acc_to_floating_base = rot_from_foot_to_floating_base * rot_from_acc_to_foot;
        // Compute rot_from_floating_base_to_world
        wbi::Rotation wbi_rot_from_world_to_sensor;
        // Passing the transpose just because of the different storing order of the matrices
        wbi_rot_from_world_to_sensor.setDcm(rot_from_world_to_sensor.transpose().data());
//        std::cerr << "[floatinBase::compute_Rot_from_floatingBase_to_world] rot_from_world_to_sensor " << std::endl << rot_from_world_to_sensor << std::endl;
//        std::cerr << "[floatinBase::compute_Rot_from_floatingBase_to_world] rot_from_acc_to_floating_base " << std::endl
//        << rot_from_acc_to_floating_base.toString().c_str() << std::endl;
        wbi::Rotation wbi_rot_from_floating_base_to_world = rot_from_acc_to_floating_base * wbi_rot_from_world_to_sensor;
        // The previous line gives actually the rotation from <world> to <floatingBase>. Therefore the following inverse (Transpose)
        wbi_rot_from_floating_base_to_world.setToInverse();
        //FIXME: I need to this in a smarter way
        for (unsigned int i = 0; i < 3; i++)
        {
            for (unsigned int j = 0; j < 3; j++)
            {
                rot_from_floatingBase_to_world(i+1,j+1) = wbi_rot_from_floating_base_to_world(i,j);
            }
        }
        //std::cerr << "wbi_rot_from_floating_base_to_world: " << wbi_rot_from_floating_base_to_world.toString().c_str() << std::endl;
        //std::cerr << "rot_from_floatingBase_to_world: " << rot_from_floatingBase_to_world << std::endl;

        return true;
    }


    bool floatingBase::getWbiOptionsAndModelJoints(yarp::os::ResourceFinder &rf,
                                                   wbi::IDList &RobotDynamicModelJoints,
                                                   yarp::os::Property &yarpWbiOptions)
    {
        std::string wbiConfFile;

        // Find WBI configuration file and retrieve all options specified.
        if (!rf.check("wbi_conf_file"))
        {
            yError("WBI configuration file name not specified in config file of this module.");
            return false;
        } else
        {
            wbiConfFile = rf.findFile("wbi_conf_file");
            if ( !yarpWbiOptions.fromConfigFile(wbiConfFile) )
            {
                yError("File %s does not exist and could not be read", wbiConfFile.c_str());
                return false;
            }
        }


        // Configure yarpWholeBodySensors before passing it to WholeBodyEstimatorThread
        // Get model joints list
        std::string modelJointsListName = rf.check("joints_list",
                                                   yarp::os::Value("ROBOT_DYNAMIC_MODEL_JOINTS"),
                                                   "Name of the list of joint used for the current robot").asString().c_str();
        if( !yarpWbi::loadIdListFromConfig(modelJointsListName,rf,RobotDynamicModelJoints) )
        {
            if( !yarpWbi::loadIdListFromConfig(modelJointsListName,yarpWbiOptions,RobotDynamicModelJoints) )
            {
                yError("[wholeBodyEstimatorModule::configure] Impossible to load wbiId joint list with name %s\n",modelJointsListName.c_str());
                return false;
            }
        }


        return true;
    }




}
